try:
    import pyqtgraph
    from pyqtgraph.Qt import QtGui
    print('pyqtgraph ok', pyqtgraph.__version__)
except Exception as e:
    import traceback; traceback.print_exc()