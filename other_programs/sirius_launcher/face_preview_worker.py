"""Read only the named Face window's compositor pixmap, never the screen/root.

Runs in a bounded subprocess: unavailable/Wayland-only capture fails closed.
No window focus, desktop capture, Face commands or ROS operations are performed.
"""
import ctypes as C
from ctypes.util import find_library
import io
import sys

from PIL import Image


class XImage(C.Structure):
    _fields_ = [('width', C.c_int), ('height', C.c_int), ('xoffset', C.c_int),
                ('format', C.c_int), ('data', C.c_void_p), ('byte_order', C.c_int),
                ('bitmap_unit', C.c_int), ('bitmap_bit_order', C.c_int),
                ('bitmap_pad', C.c_int), ('depth', C.c_int), ('bytes_per_line', C.c_int),
                ('bits_per_pixel', C.c_int), ('red_mask', C.c_ulong),
                ('green_mask', C.c_ulong), ('blue_mask', C.c_ulong)]


class Visual(C.Structure):
    _fields_ = [('ext_data',C.c_void_p),('visualid',C.c_ulong),('class_',C.c_int),
                ('red_mask',C.c_ulong),('green_mask',C.c_ulong),('blue_mask',C.c_ulong)]


def capture(window):
    if window <= 0:
        raise ValueError('Invalid Face window')
    x = C.CDLL(find_library('X11'))
    composite = C.CDLL(find_library('Xcomposite'))
    x.XOpenDisplay.argtypes = [C.c_char_p]; x.XOpenDisplay.restype = C.c_void_p
    x.XCloseDisplay.argtypes = [C.c_void_p]
    x.XDefaultRootWindow.argtypes = [C.c_void_p]; x.XDefaultRootWindow.restype = C.c_ulong
    x.XDefaultScreen.argtypes = [C.c_void_p]; x.XDefaultScreen.restype = C.c_int
    x.XDefaultVisual.argtypes = [C.c_void_p,C.c_int]; x.XDefaultVisual.restype = C.POINTER(Visual)
    x.XSync.argtypes = [C.c_void_p, C.c_int]
    x.XFreePixmap.argtypes = [C.c_void_p, C.c_ulong]
    x.XDestroyImage.argtypes = [C.POINTER(XImage)]
    x.XGetGeometry.argtypes = [C.c_void_p,C.c_ulong,C.POINTER(C.c_ulong),C.POINTER(C.c_int),C.POINTER(C.c_int),C.POINTER(C.c_uint),C.POINTER(C.c_uint),C.POINTER(C.c_uint),C.POINTER(C.c_uint)]
    x.XGetImage.argtypes = [C.c_void_p,C.c_ulong,C.c_int,C.c_int,C.c_uint,C.c_uint,C.c_ulong,C.c_int]
    x.XGetImage.restype = C.POINTER(XImage)
    composite.XCompositeNameWindowPixmap.argtypes = [C.c_void_p,C.c_ulong]
    composite.XCompositeNameWindowPixmap.restype = C.c_ulong
    errors = []
    handler_type = C.CFUNCTYPE(C.c_int,C.c_void_p,C.c_void_p)
    handler = handler_type(lambda *_: errors.append(True) or 0)
    x.XSetErrorHandler.argtypes = [handler_type]; x.XSetErrorHandler(handler)
    display = x.XOpenDisplay(None)
    if not display:
        raise RuntimeError('Face display unavailable')
    pixmap = None; image = None
    try:
        if window == x.XDefaultRootWindow(display):
            raise ValueError('Root capture prohibited')
        # Uses the compositor's offscreen surface even when Launcher covers Face.
        pixmap = composite.XCompositeNameWindowPixmap(display, window)
        x.XSync(display, 0)
        if errors or not pixmap:
            raise RuntimeError('Face compositor surface unavailable')
        root=C.c_ulong();px=C.c_int();py=C.c_int();w=C.c_uint();h=C.c_uint();border=C.c_uint();depth=C.c_uint()
        ok=x.XGetGeometry(display,pixmap,C.byref(root),C.byref(px),C.byref(py),C.byref(w),C.byref(h),C.byref(border),C.byref(depth))
        if not ok or not (1 <= w.value <= 8192 and 1 <= h.value <= 8192):
            raise RuntimeError('Unsupported Face surface size')
        image=x.XGetImage(display,pixmap,0,0,w.value,h.value,C.c_ulong(-1),2)
        x.XSync(display,0)
        if errors or not image:
            raise RuntimeError('Face image unavailable')
        info=image.contents
        masks=(info.red_mask,info.green_mask,info.blue_mask)
        if masks==(0,0,0):  # XGetImage does not assign a Visual to pixmaps.
            visual=x.XDefaultVisual(display,x.XDefaultScreen(display)).contents
            masks=(visual.red_mask,visual.green_mask,visual.blue_mask)
        if info.bits_per_pixel!=32 or info.byte_order!=0 or masks!=(0xff0000,0xff00,0xff) or info.depth not in (24,32):
            raise RuntimeError('Unsupported Face pixel format')
        pixels=C.string_at(info.data,info.bytes_per_line*info.height)
        frame=Image.frombytes('RGB',(info.width,info.height),pixels,'raw','BGRX',info.bytes_per_line)
        frame.thumbnail((240,160))
        out=io.BytesIO();frame.save(out,format='JPEG',quality=75)
        return out.getvalue()
    finally:
        if image:x.XDestroyImage(image)
        if pixmap:x.XFreePixmap(display,pixmap)
        x.XCloseDisplay(display)


if __name__ == '__main__':
    try:
        sys.stdout.buffer.write(capture(int(sys.argv[1],0)))
    except Exception as error:
        print(str(error),file=sys.stderr)
        sys.exit(1)
