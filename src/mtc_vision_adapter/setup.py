from setuptools import setup
setup(name='mtc_vision_adapter',version='0.1.0',packages=['mtc_vision_adapter'],
 data_files=[('share/ament_index/resource_index/packages',['resource/mtc_vision_adapter']),
 ('share/mtc_vision_adapter',['package.xml'])],install_requires=['setuptools'],
 zip_safe=True,maintainer='vision team',maintainer_email='vision@example.invalid',
 description='Offline battery detection replay; invalid 3D poses',
 license='Apache-2.0',entry_points={'console_scripts':['replay_publisher = mtc_vision_adapter.publisher:main']})
