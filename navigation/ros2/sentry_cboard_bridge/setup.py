from setuptools import setup
setup(name='sentry_cboard_bridge', version='0.1.0',
      packages=['sentry_cboard_bridge'],
      data_files=[('share/ament_index/resource_index/packages', ['resource/sentry_cboard_bridge']),
                  ('share/sentry_cboard_bridge', ['package.xml'])],
      install_requires=['setuptools'], zip_safe=True,
      maintainer='cool-sentry', maintainer_email='maintainer@example.com',
      description='Navigation USB bridge for existing CBoard and vision USART', license='MIT',
      entry_points={'console_scripts': ['bridge = sentry_cboard_bridge.node:main']})
