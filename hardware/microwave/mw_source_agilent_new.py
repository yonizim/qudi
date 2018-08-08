# -*- coding: utf-8 -*-

"""
This file contains the Qudi hardware file to control Gigatronics Device.

Qudi is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

Qudi is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with Qudi. If not, see <http://www.gnu.org/licenses/>.

Parts of this file were developed from a PI3diamond module which is
Copyright (C) 2009 Helmut Rathgen <helmut.rathgen@gmail.com>

Copyright (c) the Qudi Developers. See the COPYRIGHT.txt file at the
top-level directory of this distribution and at <https://github.com/Ulm-IQO/qudi/>
"""

import visa
import numpy as np
import time

from core.module import Base
from interface.microwave_interface import MicrowaveInterface
from interface.microwave_interface import MicrowaveLimits
from interface.microwave_interface import MicrowaveMode
from interface.microwave_interface import TriggerEdge


class MicrowaveAgilent(Base, MicrowaveInterface):
    """ Hardware file for Agilent. """

    _modclass = 'MicrowaveInterface'
    _modtype = 'hardware'

    def on_activate(self):
        """ Initialisation performed during activation of the module.
        """
        # checking for the right configuration
        config = self.getConfiguration()
        if 'gpib_address' in config.keys():
            self._gpib_address = config['gpib_address']
        else:
            self.log.error('This is MWagilent: did not find >>gpib_address<< in configration.')

        if 'gpib_timeout' in config.keys():
            self._gpib_timeout = int(config['gpib_timeout'])
        else:
            self._gpib_timeout = 10
            self.log.error('This is MWagilent: did not find >>gpib_timeout<< in configration. '
                           'I will set it to 10 seconds.')

        # trying to load the visa connection to the module
        self.rm = visa.ResourceManager()
        try:
         #  self._gpib_connection = self.rm.open_resource(self._gpib_address,
           #                                             read_termination='\r\n',
                 #                                         timeout=self._gpib_timeout*1000)
            self._gpib_connection = self.rm.open_resource(self._gpib_address, read_termination='\n', timeout = self._gpib_timeout * 1000)
        except:
            self.log.error('This is MWagilent: could not connect to the GPIB address >>{}<<.'
                           ''.format(self._gpib_address))
            raise
        self._gpib_connection.write('*RST')
        idnlist = []
        while len(idnlist) < 3:
            idnlist = self._gpib_connection.query('*IDN?').split(', ')
            time.sleep(0.1)
        self.model = idnlist[1]
        self.log.info('MWagilent initialised and connected to hardware.')

        # Settings must be locally saved because the SCPI interface of that device is too bad to
        # query those values.
        self._freq_list = [2.8e9,2.9e9]
        self._list_power = -136
        self._cw_power = -136
        self._cw_frequency = 2870.0e6
    def on_deactivate(self):
        """ Deinitialisation performed during deactivation of the module.
        """
        self._gpib_connection.close()
        self.rm.close()
    def get_limits(self):
        """Limits of Gigatronics 2400/2500 microwave source series.

          return MicrowaveLimits: limits of the particular Gigatronics MW source model
        """
        limits = MicrowaveLimits()
        limits.supported_modes = (MicrowaveMode.CW, MicrowaveMode.LIST)

        limits.min_frequency = 300e3
        limits.max_frequency = 6.4e9

        limits.min_power = -136
        limits.max_power = 10

        limits.list_minstep = 0.1
        limits.list_maxstep = 6.4e9
        limits.list_maxentries = 4000

        limits.sweep_minstep = 0.1
        limits.sweep_maxstep = 6.4e9
        limits.sweep_maxentries = 10001

        return limits
    def _command_wait(self, command_str):
        
        """Writes the command in command_str via GPIB and waits until the device has finished 
        processing it.

        @param command_str: The command to be written"""""
        
        self._gpib_connection.write(command_str)
        self._gpib_connection.write('*WAI')
        while int(float(self._gpib_connection.query('*OPC?'))) != 1:
            time.sleep(0.2)
        return
    def off(self):
        """ 
        Switches off any microwave output.
        Must return AFTER the device is actually stopped.

        @return int: error code (0:OK, -1:error)
        """
        self._gpib_connection.write(':Output:state 0')
        while int(float(self._gpib_connection.query(':Output:state?'))) != 0:
            time.sleep(0.2)
        return 0
    def get_status(self):
        """ 
        Gets the current status of the MW source, i.e. the mode (cw, list or sweep) and 
        the output state (stopped, running)

        @return str, bool: mode ['cw', 'list', 'sweep'], is_running [True, False] 
        """
        is_running = bool(int(float(self._gpib_connection.query(':Output?'))))
        mode = self._gpib_connection.query(':Freq:mode?').strip('\n').lower()
        return mode, is_running
    def get_power(self):
        """ 
        Gets the microwave output power. 

        @return float: the power set at the device in dBm
        """
        mode, dummy = self.get_status()
        if mode == 'list':
            return self._list_power
        else:
            return float(self._gpib_connection.query(':POW?'))
    def get_frequency(self):
        """ 
        Gets the frequency of the microwave output.
        Returns single float value if the device is in cw mode. 
        Returns list like [start, stop, step] if the device is in sweep mode.
        Returns list of frequencies if the device is in list mode.

        @return [float, list]: frequency(s) currently set for this device in Hz
        """
        mode, is_running = self.get_status()
        if 'c' in mode:
            return_val = float(self._gpib_connection.query(':Freq?'))
        elif 'lis' in mode:
            return_val = self._freq_list
        else:
            return_val = -1
        return return_val
    def cw_on(self):
        """ Switches on any preconfigured microwave output.

        @return int: error code (0:OK, -1:error)
        """
        mode, is_running = self.get_status()
        if is_running:
            if mode == 'cw':
                return 0
            else:
                self.off()

        if mode != 'cw':
            self.set_cw()

        self._gpib_connection.write(':Output:state 1')
        dummy, is_running = self.get_status()
        while not is_running:
            time.sleep(0.2)
            dummy, is_running = self.get_status()
        return 0
    def set_cw(self, frequency=None, power=None):
        """ 
        Configures the device for cw-mode and optionally sets frequency and/or power

        @param float frequency: frequency to set in Hz
        @param float power: power to set in dBm

        @return float, float, str: current frequency in Hz, current power in dBm, current mode
        """
        mode, is_running = self.get_status()
        if is_running:
            self.off()

        if mode != 'cw':
            self._command_wait(':MODE CW')

        if frequency is not None:
            self._command_wait(':FREQ {0:e}'.format(frequency))
        else:
            self._command_wait(':FREQ {0:e}'.format(self._cw_frequency))

        if power is not None:
            self._command_wait(':POW {0:f} DBM'.format(power))
        else:
            self._command_wait(':POW {0:f} DBM'.format(self._cw_power))

        mode, dummy = self.get_status()
        self._cw_frequency = self.get_frequency()
        self._cw_power = self.get_power()
        return self._cw_frequency, self._cw_power, mode
    def list_on(self):
        """
        Switches on the list mode microwave output.
        Must return AFTER the device is actually running.

        @return int: error code (0:OK, -1:error)
        """
        mode, is_running = self.get_status()
        if is_running:
            if mode == 'list':
                return 0
            else:
                self.off()

        if mode != 'list':
            self.set_list()

        self._gpib_connection.write(':Output:state 1')
        #self._gpib_connection.write(':Initiate:continuous ON')
        dummy, is_running = self.get_status()
        while not is_running:
            time.sleep(0.2)
            dummy, is_running = self.get_status()
        return 0
    def set_list(self, frequency=None, power=None):
        """ 
        Configures the device for list-mode and optionally sets frequencies and/or power

        @param list frequency: list of frequencies in Hz
        @param float power: MW power of the frequency list in dBm

        @return list, float, str: current frequencies in Hz, current power in dBm, current mode
        """

        dwell = 0.030 #in seconds. Should be less than the triggger - IS IT TRUE ??
        mode, is_running = self.get_status()

        if is_running:
            self.off()

        old_cw_power = self._cw_power
        old_cw_frequency = self._cw_frequency

        if frequency is not None:
            frequency_cw = frequency[0]
        else:
            frequency_cw = self._freq_list[0]
        if power is not None:
            if type(power) is list:
                power_cw = power[0]
            else:
                power_cw = power
        else:
            power_cw = self._list_power
        self.set_cw(frequency_cw, power_cw)

        #if frequency is not None:
        #    self.set_cw(frequency=frequency[0])
        #else:
        #    self.set_cw(frequency=self._freq_list[0])
        #if power is not None:
        #    self.set_cw(power=power[0])
        #else:
        #    self.set_cw(power=self._list_power)

        self._cw_power = old_cw_power
        self._cw_frequency = old_cw_frequency

        #self._gpib_connection.write('*SRE 0')
        self._gpib_connection.write(':LIST:SEQ:AUTO ON')

        if frequency is not None:
            freqstring = '{0:.1f} Hz, '.format(frequency[0]) + ','.join((' {0:.1f} Hz'.format(f) for f in frequency[1:]))
            dwestring = '{0:.3f} s, '.format(dwell) + ','.join((' {0:.3f} s'.format(dwell) for f in frequency[1:]))
            self._freq_list = frequency
        else:
            freqstring = '{0:.1f} Hz, '.format(self._freq_list[0]) + ','.join((' {0:.1f} Hz'.format(f) for f in self._freq_list[1:]))
            dwestring = '{0:.3f} s, '.format(dwell)
        #self._gpib_connection.write('LIST:FREQ {0:s}'.format(freqstring))

        if power is not None:
            if type(power) is list:
                powstring = '{0:.3f} dbm, '.format(power[0]) + ','.join((' {0:.3f} dbm'.format(p) for p in power[1:]))
            else:
                powstring = '{0:.3f} dbm, '.format(power) + ','.join((' {0:.3f} dbm'.format(power) for f in frequency[1:]))
            self._list_power = power
        else:
            powstring = '{0:.3f} dbm'.format(self._list_power)
            powstring = powstring + len(self._freq_list) * ', {0:.3f} dbm'.format(self._list_power)


        self._gpib_connection.write(':LIST:Type List')
        self._gpib_connection.write(':Output 0;:Freq:mode List;:Pow:mode FIX;:LIST:FREQ ' + freqstring + ';:LIST:POW ' + powstring + ';:LIST:DWELL ' + dwestring )
        self._gpib_connection.write(':INIT:CONT 1');
        self._gpib_connection.write(':SWEEP:TRIG:source EXT')

        #self._gpib_connection.write(':Pow %f dbm' % power)

        mode, dummy = self.get_status()
        return self._freq_list, self._list_power, mode
    def reset_listpos(self):#
        """ Reset of MW List Mode position to start from first given frequency

        @return int: error code (0:OK, -1:error)
        """
        pass
         # to do  - I changed the code!!!
        #self._gpib_connection.write(':MODE LIST')
        #self._gpib_connection.write('*WAI')
        #mode, is_running = self.get_status()
        #return 0 if ('list' in mode) and is_running else -1
        return 0
    def set_ext_trigger(self, pol=TriggerEdge.RISING):
        """ Set the external trigger for this device with proper polarization.

        @param TriggerEdge pol: polarisation of the trigger (basically rising edge or
                        falling edge)

        @return int: error code (0:OK, -1:error)
        """
        self._gpib_connection.write(':TRIGGER:Source Ext')
        return TriggerEdge.RISING
    def sweep_on(self):
        """ Switches on the sweep mode.

        @return int: error code (0:OK, -1:error)
        """
        return -1
    def set_sweep(self, start=None, stop=None, step=None, power=None):
        """ 
        Configures the device for sweep-mode and optionally sets frequency start/stop/step 
        and/or power

        @return float, float, float, float, str: current start frequency in Hz, 
                                                 current stop frequency in Hz,
                                                 current frequency step in Hz,
                                                 current power in dBm, 
                                                 current mode
        """
        return -1, -1, -1, -1, ''
    def reset_sweeppos(self):
        """ 
        Reset of MW sweep mode position to start (start frequency)

        @return int: error code (0:OK, -1:error)
        """
        return -1

