from setuptools import setup, find_packages

setup(name='rpi_robotics',
      version='0.1',
      description='Raspberry Pi based rpi_robotics',
      url='https://github.com/tng-spacecamp/rpi_robotics',
      author='',
      author_email='',
      license='TBD',
      packages=find_packages(),
      zip_safe=False,
      install_requires=['numpy',
                        'picamera'])
