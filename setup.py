from setuptools import setup

setup(name='rover',
      version='0.1',
      description='Raspberry Pi based rover',
      url='https://github.com/tng-spacecamp/rover',
      author='',
      author_email='',
      license='TBD',
      packages=['rover'],
      install_required=[
          'cv2',
      ],
      zip_safe=False)
