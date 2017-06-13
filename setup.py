from setuptools import setup, find_packages

setup(name='rpibotics',
      version='0.1',
      description='Raspberry Pi based rpibotics',
      url='https://github.com/tng-spacecamp/rpibotics',
      author='',
      author_email='',
      license='TBD',
      packages=find_packages(),
      zip_safe=False,
      install_requires=['numpy',
                        'picamera',
                        'RPi.GPIO',
                        'typing'])
