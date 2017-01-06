'''
Created on Sep 10, 2012

@author: miips640
'''

import ctypes
from ctypes import c_int, WinDLL, pointer, c_float, cast, POINTER, c_ushort, c_long, c_uint, c_ulong
#from binascii import hexlify
from time import sleep
import camera_constants as consts
from struct import unpack


TRUE = 1
FALSE = 0
import win32event as event#@UnresolvedImport
import pywintypes #@UnresolvedImport
#from code import interact
import os
import threading

#from Hardware.Camera.camera_constants import DRV_SUCCESS

camlib = None



#camlib = WinDLL("ATMCD32M.lib")
libc = ctypes.cdll.msvcrt

#print os.path.dirname(__file__)

DEFUALT_TEMPERATURE = -40
MINIMAL_SHUTDOWN_TEMP = -20

class  CameraControl :
    
    def __init__(self, callbackFunc = None, debug = True):
        
        global camlib
        if camlib == None :
            if debug: print 'Loading Camera driver...'
            dllpath = os.path.dirname(__file__) + "\\atmcd32d.dll"
            if debug: print 'Searching for ', dllpath
            camlib = WinDLL(dllpath)
            if debug: print camlib
        
        print 'Initializing Camera...'
        
        self.debug = debug
        self._pixelsX = c_int(1)
        self._pixelsY = c_int(1)
        
        if camlib.Initialize("") != consts.DRV_SUCCESS :
            print "Initialization failed"
        else :
            print "Initialization Successfull"
            
        headModel = ctypes.create_string_buffer('Hello', consts.MAX_PATH)
        if camlib.GetHeadModel(headModel) != consts.DRV_SUCCESS :
            print "Error Getting head model"
        else:
            print "Head model: ", headModel.raw
        
        serialNumber = c_int(-1)
        assert camlib.GetCameraSerialNumber(pointer(serialNumber)) == consts.DRV_SUCCESS
        self._serialNumber = serialNumber.value
        print 'Serial Number: ', self._serialNumber
        
        HW = [];
        for i in range(6):
            HW.append(c_int(i))    
        ans = camlib.GetHardwareVersion(
                pointer(HW[0]), pointer(HW[1]), pointer(HW[2]),
                pointer(HW[3]), pointer(HW[4]), pointer(HW[5]))
        if ans != consts.DRV_SUCCESS :
            print "Error Getting Hardware Version."
        else: 
            print 'Hardware information: ', HW
        
        SW = [];
        for i in range(6):
            SW.append(c_int(i))    
        ans = camlib.GetHardwareVersion(
                pointer(SW[0]), pointer(SW[1]), pointer(SW[2]),
                pointer(SW[3]), pointer(SW[4]), pointer(SW[5]))
        if ans != consts.DRV_SUCCESS :
            print "Error Getting Software Version."
        else: 
            print 'Software information: ', SW
        
        ans = camlib.GetDetector(pointer(self._pixelsX), pointer(self._pixelsY))
        if ans != consts.DRV_SUCCESS :
            print "Couldn't get dimensions."
        else :
            print "Dimensions: ", self._pixelsX.value, self._pixelsY.value
        
        numADChan = c_int(-1)
        assert camlib.GetNumberADChannels(pointer(numADChan)) == consts.DRV_SUCCESS
        print '# of AD channels [expecting one]: ', numADChan.value
#        self._numADChan = numADChan.value
        assert camlib.SetADChannel(0) == consts.DRV_SUCCESS
        print 'Set as active channel'
        
        ampNum = c_int(-1)
        assert camlib.GetNumberAmp(pointer(ampNum)) == consts.DRV_SUCCESS
        print 'Number of output amplifiers: ', ampNum.value
        
        numHSSpeeds = c_int(-1)
        assert camlib.GetNumberHSSpeeds(0, 0, pointer(numHSSpeeds)) == consts.DRV_SUCCESS
        print '# of horizontal speeds: ', numHSSpeeds.value
        self._HSSpeeds = []
        speed = c_float(0)
        for i in range(numHSSpeeds.value):
            assert camlib.GetHSSpeed(0, 0, i, pointer(speed)) == consts.DRV_SUCCESS
            self._HSSpeeds.append(speed.value)
        print 'Horizontal speeds [MHz]: ', self._HSSpeeds
        assert camlib.SetHSSpeed(0, 0) == consts.DRV_SUCCESS
        print 'Horizontal speed set to maximum.'
        
        numVSSpeeds = c_int(-1)
        assert camlib.GetNumberVSSpeeds(pointer(numVSSpeeds)) == consts.DRV_SUCCESS
        print '# of vertical speeds: ', numVSSpeeds.value
        self._VSSpeeds = []
        for i in range(numVSSpeeds.value):
            assert camlib.GetVSSpeed(i, pointer(speed)) == consts.DRV_SUCCESS
            self._VSSpeeds.append(speed.value)
        print 'Vertical speeds [microseconds per pixel shift]: ' , self._VSSpeeds
        index = c_int(-1)
        assert camlib.GetFastestRecommendedVSSpeed(pointer(index), pointer(speed)) == consts.DRV_SUCCESS
        print ('Recommended speed is %f, at index %d' % ( speed.value, index.value))
        assert camlib.SetVSSpeed(index) == consts.DRV_SUCCESS
        print 'Vertical speed set to recommended value.'
        
        numGains = c_int(-1)
        assert camlib.GetNumberPreAmpGains(pointer(numGains)) == consts.DRV_SUCCESS
        print '# of gains: ', numGains.value
        self._preampGains = []
        gain = c_float(-1);
        for i in range(numGains.value) :
            assert camlib.GetPreAmpGain(i, pointer(gain)) == consts.DRV_SUCCESS
            self._preampGains.append(gain.value)
        print 'Preamp gains available: ', self._preampGains
        ind = 1
        assert camlib.SetPreAmpGain(ind) == consts.DRV_SUCCESS
        print 'Preamp gain set to  = ', self._preampGains[ind]
        assert camlib.SetPreAmpGain(0) == consts.DRV_SUCCESS
        
        self._minTemp = c_int(0)
        self._maxTemp = c_int(0)
        ans = camlib.GetTemperatureRange(pointer(self._minTemp), pointer(self._maxTemp))       
        if ans != consts.DRV_SUCCESS :
            print "Error getting acceptable temperature range."
        else:
            print "Acceptable Temperatures: ", self._minTemp.value, self._maxTemp.value
            
            
        self._lastTemp = c_int(0)
        ans = camlib.GetTemperature(pointer(self._lastTemp))
        if ans == consts.DRV_NOT_INITIALIZED or ans == consts.DRV_ERROR_ACK :
            print "Error getting current temperature"
        else:
            print "Current Temperature", self._lastTemp.value
        
        
        if camlib.SetTemperature(DEFUALT_TEMPERATURE) != consts.DRV_SUCCESS:
            print "Error setting desired temperature."
        else:
            print "Operating temperature set to ", DEFUALT_TEMPERATURE
        
        if camlib.CoolerON() != consts.DRV_SUCCESS :
            print "Failed to start cooler."
        else :
            print "Cooler On."
            
#        print "Waiting for device to cool down"
#        t = self.getTemperature()
#        while (t > DEFUALT_TEMPERATURE) :
#            sleep(3)
#            t = self.getTemperature();
#            if self.debug : print "Cooling: ", t
#        
#        print "Device sufficiently cool for operations"
        
        self._setAcquisitionParameters()
        self._callbackFunc = callbackFunc
        self._waiting = False
        self._callbackParams = None
        self._setCallbackThread()
        return
    
    def setCallbackFunction(self, func):
        self._callbackFunc = func
    
    def setCallbackParameters(self, params):
        self._callbackParams = params
    
    def _setCallbackThread(self):
        
        self._alive = True
        self._acquiring = False
        
        self._callbackHandle = event.CreateEvent(None, TRUE, False, "CallbackHandle")
        event.ResetEvent(self._callbackHandle)
        print self._callbackHandle
        
        assert camlib.SetDriverEvent(c_int(self._callbackHandle)) == consts.DRV_SUCCESS
              
        self._callbackThread = threading.Thread(target = self._waitForCallback)
        self._callbackThread.start()
        return
    
    def _waitForCallback(self):
        print 'Waiting for my callback...'
        
        while self._alive :
            
            if not self._acquiring :
                continue
            
            status = event.WaitForSingleObject(self._callbackHandle, 1000)
            self.getStatus()
            if self.debug and status == event.WAIT_TIMEOUT :
                print 'callback timeout'
            if status == event.WAIT_OBJECT_0 :
                self._waiting = False
#                if self.debug : print 'called back'
                if self._callbackFunc != None :
                    if self._callbackParams == None :
                        self._callbackFunc()
                    else :
                        self._callbackFunc(self._callbackParams)
                
            event.ResetEvent(self._callbackHandle)
            
        return
    
    def _onCallback(self):
        if self._callbackFunc != None :
            self._callbackFunc()
        return
    
    def setSingleTrack(self, center, width = 1):
        if self.debug : print 'Setting center to %d, width to %d' % (center, width)
        assert camlib.SetSingleTrack(c_int(center), c_int(width)) == consts.DRV_SUCCESS
               
    def getBufferSize(self):
        size = c_long(-1)
        assert camlib.GetSizeOfCircularBuffer(pointer(size)) == consts.DRV_SUCCESS
        return size.value
    
    
    def getOutputHeight(self):
        return self._outputHeight
    
    def setFVB(self):
        assert camlib.SetReadMode(0) == consts.DRV_SUCCESS
        self._outputHeight = 1
        return
    
    def _setAcquisitionParameters(self):
        
#        print 'Setting acquisition mode to single scan.'
#        assert camlib.SetAcquisitionMode(1) == consts.DRV_SUCCESS
        
        print 'Setting exposure time to 0.0001 seconds'
        self.setExposure(0.00001)
        
        print 'Setting readout mode to full vertical binnig'
        self.setFVB()
        
#        print 'Setting Crop Mode: Bottom Five Rows'
#        assert camlib.SetIsolatedCropMode(1, 1, self.getWidth() , 1, 1) == consts.DRV_SUCCESS
        
        width = 1
        print 'Setting readout mode to single track, mid-camera, %d pixels wide' % width
        assert camlib.SetReadMode(3) == consts.DRV_SUCCESS
        assert camlib.SetSingleTrack(c_int(self.getHeight()/2), c_int(width)) == consts.DRV_SUCCESS
#        
        print 'Setting trigger mode to internal'
        assert camlib.SetTriggerMode(0) == consts.DRV_SUCCESS
        
        print 'Setting shutter to automatic mode'
        assert camlib.SetShutter(0, 0, 0, 0) == consts.DRV_SUCCESS
        
        self.setKineticSeries(0.001, 1000)
        print 'Setting acquisition mode to kinetic. %d frames %.3e seconds apart.' % (self.getNumKineticFrames(), self.getKineticCycle())
        
        
        print 'Eventual exposure time is %e seconds' % self._exposureTime
        
        self._setBuffers()
        
        print 'Setting callback minimal time to 1 frame,  0.1 ms.'
        assert camlib.SetDMAParameters(c_int(1), c_float(0.0001)) == consts.DRV_SUCCESS
        
        print 'Buffer Size is: ', self.getBufferSize()
        print 'Readout time is: ', self.getReadoutTime()
        
        return
    
    def _setBuffers(self):
        self.pixels = self.getWidth() * self.getNumKineticFrames()
        self.seriesBuffer = ctypes.create_string_buffer(2 * self.pixels)
        self.frameBuffer = ctypes.create_string_buffer(2 * self.getWidth())
        self.int_array_pointer = cast(self.seriesBuffer, POINTER(c_ushort * self.pixels))[0]
        
    def setSingleScanMode(self):
        assert camlib.SetAcquisitionMode(1) == consts.DRV_SUCCESS
    
    def setKineticCycle(self, dt):
        assert camlib.SetKineticCycleTime(c_float(dt)) == consts.DRV_SUCCESS
        self._updateTiminigs()
        return self.getKineticCycle()
        
    def setKineticSeries(self, dt, numFrames):
    
        assert camlib.SetAcquisitionMode(3) == consts.DRV_SUCCESS
        assert camlib.SetKineticCycleTime(c_float(dt)) == consts.DRV_SUCCESS
        assert camlib.SetNumberKinetics(numFrames) == consts.DRV_SUCCESS
        self._numKinetic = numFrames
        self._updateTiminigs()
        self._setBuffers()
        
        if self.debug: print 'Set Kinetic: %d frames, %.2e cycle time' % (self._numKinetic, self._kineticTIme)
        
        return self.getKineticCycle()
    
    def getKineticCycle(self):
        return self._kineticTIme
    
    def getNumKineticFrames(self):
        return self._numKinetic
    
    def getReadoutTime(self):
        readTime = c_float(-1)
        assert camlib.GetReadOutTime(pointer(readTime)) == consts.DRV_SUCCESS
        return readTime.value
    
    def getExposure(self):
        return self._exposureTime
    
    def getKineticTime(self):
        return self._kineticTIme
    
    def setExposure(self, dt):
        assert camlib.SetExposureTime(c_float(dt)) == consts.DRV_SUCCESS
        self._updateTiminigs()
        if self.debug : print 'set exposure to: ', self._exposureTime    
        return self._exposureTime
    
    def _updateTiminigs(self):
        expt = c_float(-1);
        accumt = c_float(-1);
        kinetict = c_float(-1);
        
        assert camlib.GetAcquisitionTimings(pointer(expt), pointer(accumt), pointer(kinetict)) == consts.DRV_SUCCESS
        
        self._exposureTime = expt.value;
        self._accumualteTime = accumt.value;
        self._kineticTIme = kinetict.value;
    
    def getTemperature(self):
        ans = camlib.GetTemperature(pointer(self._lastTemp))
        assert ans != consts.DRV_NOT_INITIALIZED and ans != consts.DRV_ERROR_ACK      
        return self._lastTemp.value
    
    def getAcquisitionProgress(self):
        accum = c_long(-1)
        kin = c_long(-2)
        
        assert camlib.GetAcquisitionProgress(pointer(accum), pointer(kin)) == consts.DRV_SUCCESS
        return (accum.value, kin.value)
    
    def getStatus(self):
        status = c_int(-17);
        assert camlib.GetStatus(pointer(status)) == consts.DRV_SUCCESS
        
        if status.value == consts.DRV_ACQUIRING :
            self._acquiring = True
        elif status.value == consts.DRV_IDLE :
            self._acquiring = False
            
        return status.value
        
        
    def waitForAllData(self):
        status = c_int(-17);
        pstatus = pointer(status)
        scs = consts.DRV_SUCCESS
        acq =  consts.DRV_ACQUIRING
#        exp = self._exposureTime
        
        assert camlib.GetStatus(pstatus) == scs
        
        while status.value == acq : #or status.value == consts.DRV_TEMPCYCLE :
#            if status.value == consts.DRV_TEMPCYCLE : print 'During temperature cycle...'
#            if self.debug : print self.getAcquisitionProgress()
            assert camlib.GetStatus(pstatus) == scs
#            if self.debug : print 'Waiting for acquisition to complete...'
        
        self._acquiring = False
        assert status.value == consts.DRV_IDLE
        
    def waitForData(self):
        assert camlib.WaitForAcquisition() == consts.DRV_SUCCESS
    
    def waitForCallback(self):
        
        self._waiting = True
#        print 'waiting'
        while self._waiting :
#            self.getStatus()
            if not self._acquiring :
                return - 1
#                print 'done waiting'
#        print 'done waiting'
        return 0

    def getLastAcquisition(self): 
        ans = self._retreiveData(self.seriesBuffer)
        if ans == consts.DRV_P2INVALID : print 'Array size mismatch'
        assert (ans  == consts.DRV_SUCCESS)
#        return self._intArrPointerToTuple(self.int_array_pointer)
        return self._binToNum16(self.seriesBuffer)
    
    def _retreiveData(self, data):
        return camlib.GetAcquiredData16(data, c_uint(self.pixels))
    
    def getLatestImage(self, binary = False):
        ans = camlib.GetMostRecentImage16(self.frameBuffer, c_ulong(self.getWidth()))
        
        if ans == consts.DRV_P2INVALID : print 'Array size mismatch'
        assert (ans  == consts.DRV_SUCCESS)
        if binary :
            return self.frameBuffer.raw
        else :
            return self._binToNum16(self.frameBuffer)
            
    
    def acquireData(self):
#        _ = self._acquireData()
        return self._acquireData16()
    
    def _acquireData16(self):
        wait = self.waitForAllData
        num = self.getNumKineticFrames()
        num = range(num)
        self.startAcquisition()
        wait()
#        self.waitForData()
#        for i in num :
#            wait()
        return self.getLastAcquisition()

    def startAcquisition(self):
        self._acquiring = True
        assert camlib.StartAcquisition() == consts.DRV_SUCCESS
        return 
    
    def _binToNum16(self, data):
        return unpack('<%dH' % (len(data) / 2), data.raw)
    
    def _intArrPointerToTuple(self, int_array_pointer):
        vals = []
        append = vals.append
        for x in int_array_pointer :
            append(x)
        return vals
    
    def getWidth(self):
        return self._pixelsX.value
    
    def getHeight(self):
        return self._pixelsY.value
    
    def close(self, shutCooler = False):
        
        self._alive = False
        
        if shutCooler : 
            assert camlib.CoolerOFF() == consts.DRV_SUCCESS 
            print "Cooler successfully shut down"
        
            print "Waiting for device to warm to ", MINIMAL_SHUTDOWN_TEMP
            
            t = self.getTemperature()
            while (t < MINIMAL_SHUTDOWN_TEMP) :
                sleep(1)
                t = self.getTemperature()
                if self.debug: print "Warming: ", t
                print 'Device sufficiently warm. Shuting Down...'
                
                assert camlib.ShutDown() == consts.DRV_SUCCESS
                print 'Done'
                return
    
    
def mainFunc():
    import cProfile, pstats
    global cameraControl
    cameraControl = CameraControl(callbackFunc = frameFunc, debug = True)
    
    global dt
    global frames
    dt = 0.00095;
    frames = 1000
    cameraControl.setExposure(5e-4)
    cameraControl.setKineticSeries(dt, frames)
#    print 'Profiling Acquisition...'
#    cProfile.run('cameraControl.acquireData()', 'log.txt')
#    cameraControl.acquireData()
    
#    p = pstats.Stats('log.txt')
#    p.sort_stats('cumulative').print_stats(12)
#    p.sort_stats('time').print_stats(8)
#    p.print_callers()
    
    
    cameraControl.startAcquisition()
    sleep(cameraControl.getKineticCycle() * frames * 1.2)
#    cameraControl.waitForAllData()
    cameraControl.close()
    
    global count
    print count
    
    
    return

count = 0;
last = 0;
def frameFunc():
    global count
    global last
    global frames
    count = count + 1
    global cameraControl
    _, frame = cameraControl.getAcquisitionProgress()
    if frame != (last + 1) %  frames:
        print 'Skipped ', last + 1
    last = frame
#    print count

if __name__ == '__main__':
    mainFunc()

