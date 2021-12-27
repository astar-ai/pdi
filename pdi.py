
import sys
import cv2
import math
import numpy as np

#-------------------------------------------------------------------------------#

fmap = [None, None]
lmap = [[None, None], [None, None]]

cap_cols = None
cap_rows = None
img_width = None

width_now = 640
height_now = 480
thr_now = 240

ndisp_now = 32
wsize_now = 9

is_sgbm = True

margin = 15. * math.pi / 180.

#-------------------------------------------------------------------------------#

def load_parameters(param_file):
  global Kl, Dl, Rl, xil, Kr, Dr, Rr, xir, T
  global cap_cols, cap_rows, img_width

  fs = cv2.FileStorage(param_file, cv2.FILE_STORAGE_READ)

  cap_size_node = fs.getNode("cap_size")
  cap_cols = int(cap_size_node.at(0).real())
  cap_rows = int(cap_size_node.at(1).real())
  img_width = cap_cols

  Kl = fs.getNode("Kl").mat()
  Dl = fs.getNode("Dl").mat()
  Rl = fs.getNode("Rl").mat()
  xil = fs.getNode("xil").mat()

  Kr = fs.getNode("Kr").mat()
  Dr = fs.getNode("Dr").mat()
  Rr = fs.getNode("Rr").mat()
  xir = fs.getNode("xir").mat()

  T = fs.getNode("T").mat()

  img_width = cap_cols / 2

#-------------------------------------------------------------------------------#

def init_undistort_rectify_map(k, d, r, knew, xi0, size, mode):
  fx = k[0, 0]
  fy = k[1, 1]
  cx = k[0, 2]
  cy = k[1, 2]
  s  = k[0, 1]

  k1 = d[0, 0]
  k2 = d[0, 1]
  p1 = d[0, 2]
  p2 = d[0, 3]

  ki = np.linalg.inv(knew)
  ri = np.linalg.inv(r)
  kri = np.linalg.inv(np.matmul(knew, r))

  rows = size[0]
  cols = size[1]

  mapx = np.zeros((rows, cols), dtype = np.float32)
  mapy = np.zeros((rows, cols), dtype = np.float32)

  for r in range(rows):
    for c in range(cols):
      xc = 0.0
      yc = 0.0
      zc = 0.0

      if mode == 'kRectPerspective':
        cr1 = np.array([c, r, 1.])
        xc = np.dot(kri[0, :], cr1)
        yc = np.dot(kri[1, :], cr1)
        zc = np.dot(kri[2, :], cr1)

      if mode == 'kRectLonglat':
        cr1 = np.array([c, r, 1.])
        tt = np.dot(ki[0, :], cr1)
        pp = np.dot(ki[1, :], cr1)

        xn = -math.cos(tt)
        yn = -math.sin(tt) * math.cos(pp)
        zn =  math.sin(tt) * math.sin(pp)

        cr1 = np.array([xn, yn, zn])
        xc = np.dot(ri[0, :], cr1)
        yc = np.dot(ri[1, :], cr1)
        zc = np.dot(ri[2, :], cr1)

      if mode == 'kRectFisheye':
        cr1 = np.array([c, r, 1.])
        ee = np.dot(ki[0, :], cr1)
        ff = np.dot(ki[1, :], cr1)
        zz = 2. / (ee * ee + ff * ff + 1.)

        xn = zz * ee
        yn = zz * ff
        zn = zz - 1.

        cr1 = np.array([xn, yn, zn])
        xc = np.dot(ri[0, :], cr1)
        yc = np.dot(ri[1, :], cr1)
        zc = np.dot(ri[2, :], cr1)


      if mode == 'kRectCylindrical':
        cr1 = np.array([c, r, 1.])
        tt = np.dot(ki[0, :], cr1)
        pp = np.dot(ki[1, :], cr1) + margin

        xc = -math.sin(pp) * math.cos(tt)
        yc = -math.cos(pp)
        zc =  math.sin(pp) * math.sin(tt)


      if zc < 0.0:
        mapx[r, c] = np.float32(-1.)
        mapy[r, c] = np.float32(-1.)

        continue

      rr = math.sqrt(xc * xc + yc * yc + zc * zc)
      xs = xc / rr
      ys = yc / rr
      zs = zc / rr

      xu = xs / (zs + xi0)
      yu = ys / (zs + xi0)

      r2 = xu * xu + yu * yu
      r4 = r2 * r2
      xd = (1 + k1 * r2 + k2 * r4) * xu + 2 * p1 * xu * yu + p2 * (r2 + 2 * xu * xu)
      yd = (1 + k1 * r2 + k2 * r4) * yu + 2 * p2 * xu * yu + p1 * (r2 + 2 * yu * yu)

      u = fx * xd + s * yd + cx
      v = fy * yd + cy

      mapx[r, c] = np.float32(u)
      mapy[r, c] = np.float32(v)

  return mapx, mapy

#-------------------------------------------------------------------------------#

def init_rectify_map():
  global mode, fmap, lmap, Kfisheye

  ### fisheye
  Kfisheye = np.identity(3, dtype = np.float64)
  Kfisheye[0, 0] = Kl[0, 0] * 0.4
  Kfisheye[1, 1] = Kfisheye[0, 0]
  Kfisheye[0, 2] = width_now / 2 - 0.5
  Kfisheye[1, 2] = height_now / 2 - 0.5

  ### longlat
  kRectLonglat = np.identity(3, dtype = np.float64)
  kRectLonglat[0, 0] = width_now / math.pi
  kRectLonglat[1, 1] = height_now / math.pi

  img_size = [height_now, width_now]

  print("Initialize fisheye rectify map")
  fmap[0], fmap[1] = init_undistort_rectify_map(
      Kl, Dl, Rl, Kfisheye, xil, img_size, 'kRectFisheye')

  global kii
  kii = np.linalg.inv(Kfisheye)

  print("Initialize left longlat rectify map")
  lmap[0][0], lmap[0][1] = init_undistort_rectify_map(
      Kl, Dl, Rl, kRectLonglat, xil, img_size, 'kRectLonglat')

  print("Initialize right longlat rectify map")
  lmap[1][0], lmap[1][1] = init_undistort_rectify_map(
      Kr, Dr, Rr, kRectLonglat, xir, img_size, 'kRectLonglat')

#-------------------------------------------------------------------------------#

def disparity_image(rect_imgl, rect_imgr):
  if is_sgbm == True:
    stereo = cv2.StereoSGBM_create(0, ndisp_now, wsize_now, 
                                   3 * 8 * wsize_now * wsize_now, 
                                   3 * 32 * wsize_now * wsize_now)
  else:
    stereo = cv2.StereoBM_create(ndisp_now, wsize_now)

  disparity = stereo.compute(rect_imgl, rect_imgr)
  disparity = disparity / 16.

  return disparity

#-------------------------------------------------------------------------------#

# point cloud generation!
# Credit: Thanks Yawen Lu <yl4280@rit.edu> for providing ply saving routine

ply_header = '''ply
format ascii 1.0
element vertex %(vert_num)d
property float x
property float y
property float z
property uchar red
property uchar green
property uchar blue
end_header
'''

def write_ply(fn, disp_img, rect_imgl_fisheye):
   cx = Kfisheye[0,2]
   cy = Kfisheye[1,2]
   b1 = cv2.norm(T)

   w_pi = width_now  / math.pi
   h_pi = height_now  / math.pi
   pi_w = math.pi / width_now

   colors = np.zeros_like(rect_imgl_fisheye, dtype='f')
   points = np.zeros_like(rect_imgl_fisheye, dtype='f')
   colors_img = cv2.cvtColor(rect_imgl_fisheye, cv2.COLOR_BGR2RGB)
   rows1 = colors_img.shape[0]
   cols1 = colors_img.shape[1]

   for r in range(rows1):
     for c in range(cols1):
        if (math.hypot(c - cx, r - cy) > thr_now):
           continue

        cr2 = np.array([c, r, 1.])
        ee = np.dot(kii[0, :], cr2)
        ff = np.dot(kii[1, :], cr2)
        zz = 2. / (ee * ee + ff * ff + 1.)

        if (np.isnan(zz)):
          continue

        xn = zz * ee
        yn = zz * ff
        zn = zz - 1

        tt = math.acos(-xn)
        pp = math.acos(-yn / math.hypot(yn, zn))
        disp = disp_img[int(pp * h_pi), int(tt * w_pi)]
        if (disp <= 0.):
           continue

        diff = pi_w * disp
        mgnt = b1 * math.sin(tt - diff) / math.sin(diff)

        points[r, c] = np.array([xn, yn, zn], dtype='f') * mgnt
        colors[r, c] =  colors_img[r, c]

   points = points.reshape(-1, 3)
   colors = colors.reshape(-1, 3)
   verts = np.hstack([points, colors])
   with open(fn, 'wb') as f:
        f.write((ply_header % dict(vert_num=len(verts))).encode('utf-8'))
        np.savetxt(f, verts, fmt='%f %f %f %d %d %d ')

#-------------------------------------------------------------------------------#

def main():
  param_file = "astar_calicam.yml"
  image_name = "wm_garden.jpg"

  if len(sys.argv) == 2:
    param_file = sys.argv[1]

  if len(sys.argv) == 3:
    param_file = sys.argv[1]
    image_name = sys.argv[2]

  load_parameters(param_file)
  init_rectify_map()

  raw_img = cv2.imread(image_name, 1)

  raw_imgl = raw_img[:, : int(img_width)]
  raw_imgr = raw_img[:, int(img_width) : int(img_width * 2)]

  rect_imgl_fisheye = cv2.remap(raw_imgl, fmap[0], fmap[1], cv2.INTER_LINEAR)
  rect_imgl_longlat = cv2.remap(raw_imgl, lmap[0][0], lmap[0][1], cv2.INTER_LINEAR)
  rect_imgr_longlat = cv2.remap(raw_imgr, lmap[1][0], lmap[1][1], cv2.INTER_LINEAR)

  dim = (int(img_width / 2), int(cap_rows / 2))
  small_img = cv2.resize(raw_imgl, dim, interpolation = cv2.INTER_NEAREST)
  cv2.imshow("Raw Image", small_img)

  disp_img = disparity_image(rect_imgl_longlat, rect_imgr_longlat)

  ### point cloud generation
  out_fn = 'pdi_point_cloud.ply'

  print('Save %s' % out_fn)
  write_ply(out_fn, disp_img, rect_imgl_fisheye)

  print('Press \'q\' to finish')
  key = cv2.waitKey(0)

  if key & 0xFF == ord('q') or key  == 27:
    return

#-------------------------------------------------------------------------------#

if __name__ == "__main__":
  main()

#-------------------------------------------------------------------------------#
