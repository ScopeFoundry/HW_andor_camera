from __future__ import division, print_function
from ScopeFoundry import BaseMicroscopeApp
#from ScopeFoundry.helper_funcs import sibling_path, load_qt_ui_file

class AndorTestApp(BaseMicroscopeApp):

    name = 'andor_test'
    
    def setup(self):
        
        from ScopeFoundryHW.andor_camera import AndorCCDHW
        self.add_hardware(AndorCCDHW(self))
        
        from ScopeFoundryHW.andor_camera import AndorCCDReadoutMeasure
        self.add_measurement(AndorCCDReadoutMeasure)

if __name__ == '__main__':
    import sys
    app = AndorTestApp(sys.argv)
    sys.exit(app.exec_())