from matplotlib.ticker import FormatStrFormatter
from numpy import pi, sign

class MyFormatter(FormatStrFormatter):
    def __init__(self, fmt):
        super().__init__(fmt)
    
    def __call__(self, x, pos=None):
        s = '-' if sign(x) == -1 else ''
        x = abs(round(x*4/pi, 4))
        if x == 0:
          return '0'
        if x % 2 == 0 and x % 4 != 0:
          x /= 2
          if x == 1:
            return s+r'$\frac{\pi}{2}$'
          return s+r'$\frac{%g}{2}\pi$' % x
        elif x % 4 == 0:
          x /= 4
          if x == 1:
            return s+r'${\pi}$'
          return s+r'$%g\pi$' % x
        else:
            if x == 1:
                return s+r'$\frac{\pi}{4}$'
            return s+r'$\frac{%g}{4}\pi$' %x