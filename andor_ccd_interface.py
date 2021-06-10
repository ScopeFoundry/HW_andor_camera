from __future__ import absolute_import, print_function
import ctypes
from ctypes import c_int, c_uint, c_byte, c_ubyte, c_short, c_double, c_float, c_long
from ctypes import pointer, byref, windll, cdll
import time
import numpy as np
import os
from threading import Lock

import platform
import logging

from enum import Enum

from . import andor_ccd_consts as consts


logger = logging.getLogger(__name__)




DEFAULT_TEMPERATURE = -80
DEFAULT_EM_GAIN = 10
DEFAULT_OUTPUT_AMP = 0  # 0 is electron multiplication, 1 is conventional    


# Read modes for the EMCCD:
class AndorReadMode(Enum):
    FullVerticalBinning = 0
    MultiTrack = 1
    RandomTrack = 2
    SingleTrack = 3
    Image = 4
    

def _err(retval):
    if retval == consts.DRV_SUCCESS:
        return retval
    else:
        err_name = consts.consts_by_num.get(retval)
        raise IOError( "Andor DRV Failure {}: {}".format(retval, err_name))


class AndorCCD(object):
    
    def __init__(self, debug = False, initialize_to_defaults=True):
    
        self.debug = debug
        self.lock = Lock()
        
        
        if platform.architecture()[0] == '64bit':
            andorlibpath = None
            for path in [r"C:\Program Files\Andor Driver Pack 2\atmcd64d.dll",
                         r"C:\Program Files\Andor SOLIS\atmcd64d_legacy.dll",]:
                if os.path.exists(path):
                    andorlibpath = path
                    break
                
            if andorlibpath is None:    
                andorlibpath = str(os.path.join(os.path.dirname(__file__),"atmcd64d.dll"))
        else:
            andorlibpath = str(os.path.join(os.path.dirname(__file__),"atmcd32d.dll"))
        #print andorlibpath
         
         
         
        self.andorlib = windll.LoadLibrary(andorlibpath)

        
        if self.debug:  logger.debug("AndorCCD initializing")
            
        with self.lock: _err(self.andorlib.Initialize(''))
        if self.debug: logger.debug("Andor CCD Library Initialization Successful")
        
        self.get_head_model()
        
        self.get_serial_number()
        
        self.get_hardware_version()
        
        self.get_software_version()
        
        self.get_detector_shape()
            
        self.get_num_ad_channels()
        self.get_num_output_amplifiers()

        
        if initialize_to_defaults:
            self.set_ad_channel() #set default AD channel
            self.set_aq_single_scan() # set to single scan by default
            self.set_num_accumulations(1)
            self.set_num_kinetics(1)

        # EM gain
        self.em_mode = self.has_em_ccd()
        if self.em_mode:
            self.get_EM_gain_range()
            self.get_EMCCD_gain()
        
        #shift speeds
        self.read_shift_speeds()
        if initialize_to_defaults:
            if self.em_mode: self.set_hs_speed_em()
            self.set_hs_speed_conventional()
            self.set_vs_speed()

        # gains
        self.get_preamp_gains()
        if initialize_to_defaults:
            self.set_preamp_gain()

        # temperature        
        self.get_temperature_range()
        self.get_temperature()

        if initialize_to_defaults:
            self.set_temperature(DEFAULT_TEMPERATURE)
            self.set_cooler_on()
        
            # Initialize the camera
            self.set_shutter_open(False)             # Shutter closed
            self.set_output_amp(DEFAULT_OUTPUT_AMP)  # Default output amplifier
            
            if self.em_mode:
                self.set_EMCCD_gain(DEFAULT_EM_GAIN)     # Default EM Gain
        
    
    ##### Initialization Functions
    
    def get_head_model(self):
        headModel = ctypes.create_string_buffer(consts.MAX_PATH)
        with self.lock: _err(self.andorlib.GetHeadModel(headModel))
        self.headModel = headModel.raw.decode().strip('\x00')
        print(self.headModel)
        if self.debug: logger.debug("Head model: "+ repr(self.headModel))
        return self.headModel

    def get_serial_number(self):
        serialNumber = c_int(-1)
        with self.lock: _err(self.andorlib.GetCameraSerialNumber(byref(serialNumber))) 
        self.serialNumber = serialNumber.value
        if self.debug: logger.debug('Serial Number: %g' % self.serialNumber)
        return serialNumber.value
    
    def get_hardware_version(self):
        HW = [ c_int(i) for i in range(6) ] 
        with self.lock: _err(self.andorlib.GetHardwareVersion( *[ byref(h) for h in HW ] ))
        self.hardware_version = tuple([ h.value for h in HW])
        if self.debug: logger.debug('Hardware information: {}'.format( repr(self.hardware_version)))
        return self.hardware_version
    
    def get_software_version(self):
        SW = [ c_int(i) for i in range(6) ] 
        with self.lock: _err(self.andorlib.GetSoftwareVersion( *[byref(s) for s in SW] ))
        self.software_version = tuple([ s.value for s in SW ])
        if self.debug: logger.debug('Software information: %s' % repr(self.software_version))
        return self.software_version
    
    def get_detector_shape(self):
        """ returns number of pixels Nx, Ny"""
        pixelsX = c_int(1)
        pixelsY = c_int(1)
        
        with self.lock: _err(self.andorlib.GetDetector(byref(pixelsX), byref(pixelsY)))
        self.Nx = pixelsX.value
        self.Ny = pixelsY.value
        if self.debug: logger.debug("Dimensions: {} {}".format( self.Nx, self.Ny ))
        return self.Nx, self.Ny
    
    def get_num_ad_channels(self):
        numADChan = c_int(-1)
        retval = self.andorlib.GetNumberADChannels(byref(numADChan)) 
        assert retval == consts.DRV_SUCCESS, "Andor DRV Failure %i" % retval    
        self.numADChan = numADChan.value
        if self.debug: logger.debug( '# of AD channels [expecting one]: %g' % self.numADChan )
        return self.numADChan
    
    def get_num_output_amplifiers(self):
        ampNum = c_int(-1)
        retval = self.andorlib.GetNumberAmp(byref(ampNum))
        assert retval == consts.DRV_SUCCESS, "Andor DRV Failure %i" % retval
        self.ampNum = ampNum.value
        if self.debug: logger.debug( 'Number of output amplifiers: %g' % self.ampNum ) 
        return self.ampNum
    
    def get_preamp_gains(self):
        numGains = c_int(-1)
        with self.lock: _err(self.andorlib.GetNumberPreAmpGains(pointer(numGains)))
        if self.debug: logger.debug('# of gains: %g '% numGains.value)
        self.numGains = numGains.value
        self.preamp_gains = []
        gain = c_float(-1)
        for i in range(numGains.value) :
            with self.lock: _err(self.andorlib.GetPreAmpGain(i, byref(gain)))
            self.preamp_gains.append(gain.value)
        if self.debug: logger.debug('Preamp gains available: %s' % self.preamp_gains)
        return self.preamp_gains
        
    def has_em_ccd(self):
        try:
            self.get_EM_gain_range()
            return True
        except IOError:
            return False
    
    
    
    #####    
    
    def set_ad_channel(self,chan_i=0):
        assert chan_i in range(0,self.numADChan)
        with self.lock: _err(self.andorlib.SetADChannel(int(chan_i)))
        self.ad_chan = chan_i
        return self.ad_chan
    
    
    def create_buffer(self):
        if self.aq_mode in ('single', 'accumulate', 'run_till_abort'):
            self.buffer = np.zeros(shape=(self.Ny_ro, self.Nx_ro), dtype=np.int32 )     
        elif self.aq_mode == 'kinetic':
            self.get_num_kinetics()
            self.buffer = np.zeros(shape=(self.num_kin, self.Ny_ro, self.Nx_ro), dtype=np.int32)
        else:
            raise ValueError("Andor Unkown acq mode {}".format(self.aq_mode))
        print(self.buffer.shape)
        return self.buffer
    
    
    ##### ReadOut Modes #######################
    def set_readout_mode(self, ro_mode):
        if (ro_mode == AndorReadMode.FullVerticalBinning):
            self.set_ro_full_vertical_binning()
        elif (ro_mode == AndorReadMode.Image):
            self.set_ro_image_mode()
        elif (ro_mode == AndorReadMode.MultiTrack):
            raise NotImplementedError()
        elif (ro_mode == AndorReadMode.RandomTrack):
            raise NotImplementedError()
        elif (ro_mode == AndorReadMode.SingleTrack):
            self.set_ro_single_track(256, 20)
    
    def set_read_mode_by_name(self,name):
        read_mode_dict = dict(
            FullVerticalBinning = 0,
            MultiTrack = 1,
            RandomTrack = 2,
            SingleTrack = 3,
            Image = 4)
        
        readout_mode_id = read_mode_dict[name]
        self.set_readmode(readout_mode_id)
        
            
    def set_read_mode(self, mode_id):
        with self.lock: _err(self.andorlib.SetReadMode(mode_id))
    
    def set_ro_full_vertical_binning(self, hbin=1):
        self.ro_mode = 'FULL_VERTICAL_BINNING'
        with self.lock: _err(self.andorlib.SetReadMode(0)) # sets to FVB
        self.ro_fvb_hbin = hbin
        with self.lock: _err(self.andorlib.SetFVBHBin(self.ro_fvb_hbin))
        #self.outputHeight = 1
        self.Nx_ro = int(self.Nx/hbin)              
        self.Ny_ro = 1
        self.create_buffer()
        
    def set_ro_single_track(self, center, width = 1, hbin = 1):
        self.ro_mode = 'SINGLE_TRACK'
        with self.lock: _err(self.andorlib.SetReadMode(3))
        with self.lock: _err(self.andorlib.SetSingleTrack(c_int(center), c_int(width)) )
    
        with self.lock: _err(self.andorlib.SetSingleTrackHBin(c_int(hbin)))
                
        self.ro_st_hbin = hbin
        self.Nx_ro = int(self.Nx/hbin)              
        self.Ny_ro = 1

        self.ro_single_track_center = center
        self.ro_single_track_width = width
        
        self.create_buffer()
        
    def set_ro_multi_track(self, number, height, offset):
        # NOT YET IMPLEMENTED
        # SetReadMode(1)
        # SetMulitTrack
        raise NotImplementedError
        #returns bottom, gap
        
    def set_ro_random_track(self, positions):
        # NOT YET IMPLEMENTED
        # SetReadMode(2)
        #SetRandomTracks(numberoftracks, positionarray)
        raise NotImplementedError
    
    def set_ro_image_mode(self,hbin=1,vbin=1,hstart=1,hend=None,vstart=1,vend=None):
        self.ro_mode = 'IMG'
        with self.lock: _err(self.andorlib.SetReadMode(4))
        
        if hend is None:
            hend = self.Nx
        if vend is None:
            vend = self.Ny
        
        assert hend > hstart
        assert vend > vstart
        
        self.hbin = hbin
        self.vbin = vbin
        
        self.hstart = hstart
        self.hend   = hend
        
        self.vstart = vstart
        self.vend   = vend
        
        with self.lock: _err(self.andorlib.SetImage(c_int(hbin),   c_int(vbin), 
                          c_int(hstart), c_int(hend),
                          c_int(vstart), c_int(vend) ))
        
        self.Nx_ro = int((self.hend-self.hstart+1)/self.hbin)                
        self.Ny_ro = int((self.vend-self.vstart+1)/self.vbin)
        
        logger.debug("self.Nx_ro: {}, self.Ny_ro: {}".format( self.Nx_ro, self.Ny_ro )) 

        self.create_buffer()

    ### Function to return the binning based on the current readout mode ####
    def get_current_hbin(self):
        if self.ro_mode == 'IMG':
            return self.hbin
        elif self.ro_mode == 'SINGLE_TRACK':
            return self.ro_st_hbin
        elif self.ro_mode == 'FULL_VERTICAL_BINNING':
            return self.ro_fvb_hbin
    
    
    ##### Acquisition Modes #####
    def set_aq_mode(self, mode):
        print('set_aq_mode', mode)
        assert mode in ('single', 'accumulate', 'kinetic', 'run_till_abort')
        if mode == 'single': return self.set_aq_single_scan()
        if mode == 'accumulate': return self.set_aq_accumulate_scan()
        if mode == 'kinetic': return self.set_aq_kinetic_scan()
        if mode == 'run_till_abort': return self.set_aq_run_till_abort_scan()
        
    def get_aq_mode(self):
        return self.aq_mode
    
    def set_aq_single_scan(self, exposure=None):
        self.aq_mode = 'single'
        with self.lock: _err(self.andorlib.SetAcquisitionMode(1))
        
        if exposure is not None:
            with self.lock: _err(self.andorlib.SetExposureTime(c_float(exposure)))
        
    def set_aq_accumulate_scan(self, exposure_time=None, num_acc=None, cycle_time=None):
        self.aq_mode = 'accumulate'

        with self.lock: _err(self.andorlib.SetAcquisitionMode(2))

        if exposure_time is not None:
            with self.lock: _err(self.andorlib.SetExposureTime(c_float(exposure_time)))
        
        if num_acc is not None:
            with self.lock: _err(self.andorlib.SetNumberAccumulations(num_acc))

        # cycle_time only valid with internal trigger
        if cycle_time is not None:
            with self.lock: _err(self.andorlib.SetAccumulationCycleTime(cycle_time))

    def set_aq_kinetic_scan(self, exp_time=None, 
                            num_acc=None, acc_time=None,
                            num_kin=None, kin_time=None):
        self.aq_mode = 'kinetic'
        
        with self.lock: _err(self.andorlib.SetAcquisitionMode(3))

        if exp_time is not None:
            with self.lock: _err(self.andorlib.SetExposureTime(c_float(exp_time)))
        if num_acc is not None:
            with self.lock: _err(self.andorlib.SetNumberAccumulations(num_acc))
        if acc_time is not None:
            with self.lock: _err(self.andorlib.SetAccumulationCycleTime(acc_time))
        if num_kin is not None:
            with self.lock: _err(self.andorlib.SetNumberKinetics(num_kin))
        # kinetic cycle time only valid with internal trigger
        if kin_time is not None:
            with self.lock: _err(self.andorlib.SetKineticCycleTime(kin_time))
        print('kinetic')
        
    def set_aq_run_till_abort_scan(self):
        self.aq_mode = 'run_till_abort'
        #SetAcquistionMode(5) SetExposureTime(0.3) SetKineticCycleTime(0)
        print('set_aq_run_till_abort_scan')
        with self.lock: _err(self.andorlib.SetAcquisitionMode(5))
        print('set_aq_run_till_abort_scan')
        
    def set_aq_fast_kinetic_scan(self):
        # NOT YET IMPLEMENTED
        raise NotImplementedError()
                
    def set_aq_frame_transfer_scan(self):
        # NOT YET IMPLEMENTED
        raise NotImplementedError()
                
    
    ##### Triggering ##########
    trigger_modes = dict(
                     internal = 0,
                     external = 1,
                     external_start = 6,
                     external_exposure = 7,
                     external_fvb_em = 9,
                     software = 10)
    

    def set_trigger_mode(self, mode='internal'):
        mode = mode.lower()
        retval = self.andorlib.SetTriggerMode(self.trigger_modes[mode])
        assert retval == consts.DRV_SUCCESS, "Andor DRV Failure %i" % retval
    
    ####### Shift Speeds and Gain ##########
    
    def read_shift_speeds(self):
        # h speeds
        numHSSpeeds = c_int(-1)
        self.numHSSpeeds_EM = []
        self.numHSSpeeds_Conventional = []
        for chan_i in range(self.numADChan):
            if self.em_mode:
                retval = self.andorlib.GetNumberHSSpeeds(chan_i, 0, byref(numHSSpeeds)) # EM mode
                assert retval == consts.DRV_SUCCESS, "Andor DRV Failure %i" % retval
                self.numHSSpeeds_EM.append(numHSSpeeds.value)
                
                retval = self.andorlib.GetNumberHSSpeeds(chan_i, 1, byref(numHSSpeeds)) # conventional mode mode
                assert retval == consts.DRV_SUCCESS, "Andor DRV Failure %i" % retval
                self.numHSSpeeds_Conventional.append(numHSSpeeds.value)
            else:
                retval = self.andorlib.GetNumberHSSpeeds(chan_i, 0, byref(numHSSpeeds)) # EM mode
                assert retval == consts.DRV_SUCCESS, "Andor DRV Failure %i" % retval
                self.numHSSpeeds_Conventional.append(numHSSpeeds.value)

        logger.debug('# of horizontal speeds EM: {}'.format(self.numHSSpeeds_EM))
        logger.debug('# of horizontal speeds Conventional: {}'.format(self.numHSSpeeds_Conventional))
        
        self.HSSpeeds_EM = []
        self.HSSpeeds_Conventional = []
        speed = c_float(0)
        for chan_i in range(self.numADChan):
            self.HSSpeeds_EM.append([])
            if self.em_mode:
                hsspeeds = self.HSSpeeds_EM[chan_i]
                for i in range(self.numHSSpeeds_EM[chan_i]):
                    retval = self.andorlib.GetHSSpeed(chan_i, 0, i, byref(speed)) # EM mode
                    assert retval == consts.DRV_SUCCESS, "Andor DRV Failure %i" % retval
                    hsspeeds.append(speed.value)
                conventional_index = 1
            else:
                conventional_index = 0
            
            self.HSSpeeds_Conventional.append([])
            hsspeeds = self.HSSpeeds_Conventional[chan_i]
            for i in range(self.numHSSpeeds_Conventional[chan_i]):
                #print chan_i, i
                retval = self.andorlib.GetHSSpeed(chan_i,  conventional_index, i, byref(speed)) # Conventional mode
                assert retval == consts.DRV_SUCCESS, "Andor DRV Failure %i" % retval
                hsspeeds.append(speed.value)
            

        logger.debug('EM Horizontal speeds: {} MHz'.format(self.HSSpeeds_EM))
        logger.debug('Conventional Horizontal speeds: {} MHz'.format(self.HSSpeeds_Conventional))        
        #Vertical  speeds
        numVSSpeeds = c_int(-1)
        retval = self.andorlib.GetNumberVSSpeeds(byref(numVSSpeeds))
        if retval == 20991: # DRV_NOT_SUPPORTED, for the case of iDus IR InGaAs single line detectors
            self.numVSSpeeds = 0
        else:
            assert retval == consts.DRV_SUCCESS, "Andor DRV Failure %i" % retval
            self.numVSSpeeds = numVSSpeeds.value

        self.VSSpeeds = []
        speed = c_float(0)
        for i in range(self.numVSSpeeds):
            retval = self.andorlib.GetVSSpeed(i, byref(speed))
            assert retval == consts.DRV_SUCCESS, "Andor DRV Failure %i" % retval
            self.VSSpeeds.append(speed.value)
        if self.debug: logger.debug( 'Vertical speeds [microseconds per pixel shift]: %s' % self.VSSpeeds)
    
    def get_hs_speed_val_conventional(self, speed_index):
        pass

    def set_hs_speed_em(self,speed_index=0):
        logger.debug("set_hs_speed_em {}".format(speed_index))
        assert 0 <= speed_index < self.numHSSpeeds_EM[self.ad_chan]
        with self.lock: _err(self.andorlib.SetHSSpeed(0, speed_index)) # 0 = default speed (fastest), #arg0 -> EM mode = 0

    def set_hs_speed_conventional(self,speed_index=0):
        print("set_hs_speed_conventional", speed_index, self.ad_chan)
        assert 0 <= speed_index < self.numHSSpeeds_Conventional[self.ad_chan]
        if self.em_mode:
            with self.lock: _err(self.andorlib.SetHSSpeed(1, speed_index)) # 0 = default speed (fastest), #arg0 -> conventional = 1
        else:
            with self.lock: _err(self.andorlib.SetHSSpeed(0, speed_index)) # 0 = default speed (fastest), #arg0 -> conventional = 1

    def set_vs_speed(self, speed_index=0):
        assert 0 <= speed_index < self.numVSSpeeds
        with self.lock: _err( self.andorlib.SetVSSpeed(speed_index))
            
    def set_preamp_gain(self, gain_i = 0):
        with self.lock: _err( self.andorlib.SetPreAmpGain(gain_i) )
        self.preamp_gain_i = gain_i
        #print 'Preamp gain set to  = ', self.preamp_gain[ind]
        

    ####### Image Rotate and Flip ###########
    # done in Andor SDK (not on camera)
    
    def get_image_flip(self):
        hflip, vflip = c_int(-1), c_int(-1)
        with self.lock: _err( self.andorlib.GetImageFlip(byref(hflip), byref(vflip)))
        self.hflip = bool(hflip.value)
        self.vflip = bool(vflip.value)
        return self.hflip, self.vflip
    
    def set_image_flip(self, hflip=True, vflip=False):
        with self.lock: _err(self.andorlib.SetImageFlip( c_int(bool(hflip)), c_int(bool(vflip))))
    
    def get_image_hflip(self):
        return self.get_image_flip()[0]
    
    def set_image_hflip(self, hflipNew):
        hflipOld, vflipOld = self.get_image_flip()
        self.set_image_flip(hflipNew, vflipOld)
        logger.debug( "set_image_hflip: {}".format(hflipNew)) 
    
    def get_image_vflip(self):
        return self.get_image_flip()[1]
    
    def set_image_vflip(self, vflipNew):
        hflipOld, vflipOld = self.get_image_flip()
        self.set_image_flip(hflipOld, vflipNew)
        logger.debug(  "set_image_vflip: {}".format( vflipNew ))
        
    def set_image_rotate(self, rotate=0):
        # 0 - No rotation
        # 1 - Rotate 90 degrees clockwise
        # 2 - Rotate 90 degrees anti-clockwise
        assert rotate in [0,1,2]
        with self.lock: _err(self.andorlib.SetImageRotation(c_int(rotate)))       
        
    
    ####### Shutter Control ##########
    
    def set_shutter_auto(self):
        with self.lock: _err(self.andorlib.SetShutter(0, 0, 0, 0))
        
    def set_shutter_open(self, open=True):
        if open:
            with self.lock: _err(self.andorlib.SetShutter(0, 1, 0, 0))
        else:
            self.set_shutter_close()
            
    def set_shutter_close(self):
        with self.lock: _err(self.andorlib.SetShutter(0, 2, 0, 0))
    
    
    ####### Temperature Control ###########
    
    def set_cooler_on(self):
        with self.lock: _err(self.andorlib.CoolerON())
        self.cooler_on = True
        
    def set_cooler_off(self):
        with self.lock: _err(self.andorlib.CoolerOFF())
        self.cooler_on = False
        
    def set_cooler(self, coolerOn):
        if coolerOn:
            self.set_cooler_on()
        else:
            self.set_cooler_off()

    def get_cooler(self):
        return self.cooler_on 


    def get_temperature_range(self):
        min_t, max_t = c_int(0), c_int(0)
        with self.lock: _err(self.andorlib.GetTemperatureRange( byref(min_t), byref(max_t) ))
        self.min_temp = min_t.value
        self.max_temp = max_t.value 
        return self.min_temp, self.max_temp

    def set_temperature(self, new_temp):
        with self.lock: _err( self.andorlib.SetTemperature(c_int(new_temp)))
        self.get_temperature()

    def get_temperature(self):
        lastTemp = c_int(0)
        with self.lock:
            retval = self.andorlib.GetTemperature(byref(lastTemp))
        if retval == consts.DRV_ACQUIRING:
            raise IOError( "Camera busy acquiring" )
        elif retval in (consts.DRV_NOT_INITIALIZED, consts.DRV_ERROR_ACK):
            _err(retval)
        else:
            self.temperature = lastTemp.value
            self.temperature_status_num = retval
            return self.temperature

    temp_status_dict = {
        consts.DRV_TEMP_OFF: 'OFF',        
        consts.DRV_TEMP_NOT_STABILIZED: 'NOT_STABILIZED',        
        consts.DRV_TEMP_STABILIZED: 'STABILIZED',
        consts.DRV_TEMP_NOT_REACHED: 'NOT_REACHED',
        consts.DRV_TEMP_NOT_SUPPORTED: 'NOT_SUPPORTED',
        consts.DRV_TEMP_DRIFT: 'DRIFT',
        }

    def get_temperature_status(self):
        self.get_temperature()
        return self.temp_status_dict[self.temperature_status_num]

    """
    @property
    def temperature_status_str(self):
        DRV_TEMP_STABILIZED 
        DRV_TEMP_NOT_REACHED 
        DRV_TEMP_DRIFT 
        DRV_TEMP_NOT_STABILIZED        

    def is_temperature_stable(self):
        "call get_temperature first"
        if self.temperature_status == consts.
    """    

        
    
    
    #### Acquire ####
    
    # StartAcquisition() --> GetStatus() --> GetAcquiredData()
    
    def start_acquisition(self):
        with self.lock: _err(self.andorlib.StartAcquisition())

    def abort_acquisition(self):
        with self.lock: _err(self.andorlib.AbortAcquisition())

    _status_name_dict = {
        consts.DRV_IDLE: "IDLE",
        consts.DRV_TEMPCYCLE: "TEMPCYCLE",
        consts.DRV_ACQUIRING: "ACQUIRING",
        consts.DRV_ACCUM_TIME_NOT_MET: "ACCUM_TIME_NOT_MET",
        consts.DRV_KINETIC_TIME_NOT_MET: "KINETIC_TIME_NOT_MET",
        consts.DRV_ERROR_ACK: "ERROR_ACK",
        consts.DRV_ACQ_BUFFER: "ACQ_BUFFER",
        consts.DRV_SPOOLERROR: "SPOOLERROR",
    }
    def get_status(self):
        status = c_int(-1)
        with self.lock: _err(self.andorlib.GetStatus(byref(status)))
        self.status_id   = status.value
        self.status_name = self._status_name_dict[self.status_id]
        return self.status_name            

    
    def get_acquired_data(self):
        #print("buffer size", self.buffer.size)
        with self.lock: _err(self.andorlib.GetAcquiredData(self.buffer.ctypes.data_as(ctypes.POINTER(c_long)), c_uint(self.buffer.size)))
        return self.buffer

    
    #### Acquisition Timings #####################
    
    def get_acquisition_timings(self):
        exposure = c_float(-1)
        accum   = c_float(-1)
        kinetic = c_float(-1)
        
        with self.lock: _err(self.andorlib.GetAcquisitionTimings(byref(exposure), byref(accum), byref(kinetic)))
        
        self.exposure_time = exposure.value
        self.accumulation_time = accum.value
        self.kinetic_cycle_time = kinetic.value
        
        return self.exposure_time, self.accumulation_time, self.kinetic_cycle_time  
        
    
    def set_exposure_time(self, dt):
        with self.lock: _err(self.andorlib.SetExposureTime(c_float(dt)))
        self.get_acquisition_timings()
        if self.debug : logger.debug( 'set exposure to: {}'.format( self.exposure_time))    
        return self.exposure_time
    
    def get_exposure_time(self):
        return self.get_acquisition_timings()[0]
    
    def set_num_accumulations(self, num):
        with self.lock: _err(self.andorlib.SetNumberAccumulations(num))
        self.num_acc = num
    
    def get_num_accumulations(self):
        return self.num_acc
    
    def set_num_kinetics(self, num):
        """This function will set the number of scans (possibly accumulated scans) to be taken
            during a single acquisition sequence. This will only take effect if the acquisition mode is
            Kinetic Series"""
        print('set_num_kinetics', num)
        with self.lock: _err(self.andorlib.SetNumberKinetics(int(num)))
        self.num_kin = num
    
    def get_num_kinetics(self):
        return self.num_kin
    
    def set_accumulation_cycle_time(self, acc_time):
        with self.lock: _err(self.andorlib.SetAccumulationCycleTime(c_float(acc_time)))
    
    def set_kinetic_cycle_time(self, kin_time):
        with self.lock: _err(self.andorlib.SetKineticCycleTime(c_float(kin_time)))
    
    ###### Electron Multiplication Mode (EM) ########
    def set_EM_advanced(self, state=True):
        with self.lock: _err(self.andorlib.SetEMGainRange(c_int(state)))
        
    def get_EM_gain_range(self):
        low, high = c_int(-1), c_int(-1)
        with self.lock: _err(self.andorlib.GetEMGainRange(byref(low),byref(high)))
        self.em_gain_range = (low.value, high.value)
        return self.em_gain_range
    
    def get_EMCCD_gain(self):
        gain = c_int(-1)
        with self.lock: _err(self.andorlib.GetEMCCDGain(byref(gain)))
        self.em_gain = gain.value
        return self.em_gain
    
    def set_EMCCD_gain(self, gain):
        low,high = self.em_gain_range
        assert low <= gain <= high
        with self.lock: _err(self.andorlib.SetEMCCDGain(c_int(gain)))
    
    def set_output_amp(self, amp):
        with self.lock: _err(self.andorlib.SetOutputAmplifier(c_int(amp)))
        self.output_amp = amp
        
    def get_output_amp(self):
        return self.output_amp
    
    def close(self):
        with self.lock: _err(self.andorlib.ShutDown())
    
    def get_total_number_images_acquired(self):
        num = c_long(0)
        with self.lock: _err(self.andorlib.GetTotalNumberImagesAcquired(byref(num)))
        return num.value
    
    def get_number_new_images(self):
        """
        This function will return information on the number of 
        new images (i.e. images which have not yet been 
        retrieved) in the circular buffer. This information can
         be used with GetImages to retrieve a series of the
        latest images. If any images are overwritten in
         the circular buffer they can no longer be retrieved
         and the information returned will treat overwritten images as having been retrieved.
        """
        first = c_long(0)
        last = c_long(0)
        with self.lock: _err(self.andorlib.GetNumberNewImages(byref(first),byref(last)))
        return first.value, last.value
    
    def get_number_available_images(self):
        first = c_long(0)
        last = c_long(0)
        with self.lock: _err(self.andorlib.GetNumberAvailableImages(byref(first),byref(last)))
        return first.value, last.value
    
    def get_images(self,first,last, buf):
        validfirst = c_long(0)
        validlast = c_long(0)
        
        with self.lock: _err(self.andorlib.GetImages(c_long(first), c_long(last), 
                                buf.ctypes.data_as(ctypes.POINTER(c_long)),
                                c_uint(buf.size),
                                byref(validfirst),byref(validlast)))
        
        return validfirst, validlast, buf

    def get_oldest_image(self, arr=None):
        if arr is None:
            arr = np.zeros((self.Ny_ro, self.Nx_ro), dtype=np.int32)
         
        arr_ptr = arr.ctypes.data_as(ctypes.POINTER(c_long))
        arr_size = c_uint(arr.size)
        with self.lock:
            retval = self.andorlib.GetOldestImage(arr_ptr, arr_size)
        #print("GetOldestImage", retval)
        if retval == consts.DRV_NO_NEW_DATA: # DRV_NO_NEW_DATA
            #print("no new data")
            return None
        else:
            _err(retval)
        return arr
    
if __name__ == '__main__':
    import time
    
    cam = AndorCCD(debug=True)
    
    cam.set_ro_image_mode()
    cam.set_trigger_mode('internal')
    cam.set_exposure_time(1.0)
    #cam.set_shutter_open()
    cam.andorlib.SetOutputAmplifier(0) # EMCCD
    
    cam.read_shift_speeds()
    
    cam.andorlib.SetOutputAmplifier(1) # Conventional
    
    #cam.set_hs_speed(1)
    cam.andorlib.SetEMGainMode(1)
    print("EM_gain_range", cam.get_EM_gain_range())
    cam.start_acquisition()
    stat = "ACQUIRING",
    while stat != "IDLE":
        time.sleep(0.1)
        stati, stat = cam.get_status()
    cam.get_acquired_data()
    cam.set_shutter_close()

    import pylab as pl
    pl.imshow(cam.buffer, interpolation='nearest', origin='lower')
    pl.show()
    


