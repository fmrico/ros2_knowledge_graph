from setuptools import setup

package_name = 'ros2_knowledge_graph_viewer'
setup(
    name=package_name,
    version='0.2.0',
    package_dir={'': 'src'},
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name + '/resource', ['resource/Ros2KnowledgeGraph.ui']),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['plugin.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Francisco Martin',
    maintainer='Francisco Martin',
    maintainer_email='fmrico@gmail.com',
    keywords=['BICA'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: BSD License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description=(
        'ros2_knowledge_graph_viewer provides a GUI plugin for visualizing the BICA graph.'
    ),
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ros2_knowledge_graph_viewer = ros2_knowledge_graph_viewer.main:main',
        ],
    },
)
