from setuptools import setup
setup(name='arc_course', version='2.0.0', packages=['arc_course'],
      data_files=[('share/ament_index/resource_index/packages',['resource/arc_course']),('share/arc_course',['package.xml'])],
      install_requires=['setuptools'], zip_safe=True,
      maintainer='ARC teaching team', maintainer_email='course@example.com',
      description='Robot motion and policy experiments', license='Apache-2.0',
      entry_points={'console_scripts':['drive_distance = arc_course.drive_distance:main',
                                       'policy_driver = arc_course.policy_driver:main']})
