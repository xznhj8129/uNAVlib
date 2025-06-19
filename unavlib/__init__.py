"""YAMSPy: Yet Another Implementation of Multiwii Serial Protocol Python Interface for Betaflight, iNAV, etc.
Copyright (C) 2019 Ricardo de Azambuja
This file is (was) part of YAMSPy.
YAMSPy is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.
YAMSPy is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.
You should have received a copy of the GNU General Public License
along with YAMSPy.  If not, see <https://www.gnu.org/licenses/>.
Deeply based on code from Betaflight and iNAV.
Acknowledgement:
This work was possible thanks to the financial support from IVADO.ca (postdoctoral scholarship 2019/2020).
Disclaimer (adapted from Wikipedia):
None of the authors, contributors, supervisors, administrators, employers, friends, family, vandals, or anyone else 
connected (or not) with this project, in any way whatsoever, can be made responsible for your use of the information (code) 
contained or linked from here.
TODO:
1) Add more support to iNAV.
2) Add the possibility to register a callback functions when a msg is transmitted.
3) Improve the arming confirmation.
4) This file is way too big... it needs to be broken down into smaller ones. (donr)
"""

__author__ = "Ricardo de Azambuja"
__copyright__ = "Copyright 2019, MISTLab.ca"
__credits__ = [""]
__license__ = "GPL"
__version__ = "0.1.2"
__maintainer__ = "frogmane"
__email__ = ""
__status__ = "Development"

from .main import MSPy