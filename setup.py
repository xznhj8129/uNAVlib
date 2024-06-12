import sys
from setuptools import setup, find_packages
from unavlib import __version__

if sys.version_info < (3, 10):
    sys.exit('Sorry, Python < 3.10 is not supported.')

with open("README.md", "r") as fh:
    long_description = fh.read()
    
setup(
    name="unavlib",
    packages=[package for package in find_packages()],
    version=__version__,
    license="GPL",
    description="MultiWii Serial Protocol autonomous flight SDK for INAV",
    long_description=long_description,
    long_description_content_type="text/markdown",
    author="Frogmane",
    author_email="",
    url="https://github.com/xznhj8129/uNAVlib",
    download_url="",
    keywords=['Betaflight', 'iNAV', 'drone', 'UAV', 'Multi Wii Serial Protocol', 'MSP'],
    install_requires=['pyserial','asyncio','simple-pid','geographiclib','mgrs','geojson'],
    classifiers=[
          'Development Status :: 4 - Beta',
          'Intended Audience :: Developers',
          'Intended Audience :: Education',
          'Intended Audience :: Information Technology',
          'Intended Audience :: Science/Research',
          'License :: OSI Approved :: GNU General Public License v3 (GPLv3)',
          'Operating System :: Microsoft :: Windows',
          'Operating System :: POSIX :: Linux',
          'Programming Language :: Python',
          'Framework :: Robot Framework :: Library',
          'Topic :: Education',
          'Topic :: Scientific/Engineering :: Artificial Intelligence'
    ]
)