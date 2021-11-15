from distutils.core import setup
#from setuptools import setup
#from catkin_pkg.python_setup import generate_disutils_setup


setup(
    version='0.0.0',

    #scripts=['scripts/talker.py','scripts/listener.py', 'scripts/QC_node.py','scripts/hdEMG_DCNN.py','scripts/hdEMG_validate.py', 'scripts/qc_communication.py', 'scripts/qc_stream.py', 'scripts/test_script.py'],

    packages=['talker_listener'],

    package_dir={'':'src'}
)
'''
d = generate_disutils_setup(
    packages = ['talker_listener'],
    package_dir={'': 'scripts'}
)

setup(**d)
'''