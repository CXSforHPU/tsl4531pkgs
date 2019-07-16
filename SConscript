from building import *

cwd     = GetCurrentDir()
src     = Glob('*.c') + Glob('*.cpp')
path    = [cwd]

group = DefineGroup('tsl4531', src, depend = ['PKG_USING_TSL4531'], CPPPATH = path)

Return('group')
