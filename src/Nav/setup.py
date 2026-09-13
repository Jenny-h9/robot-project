from setuptools import setup
package_name='nav'
setup(name=package_name, version='0.1.0', packages=[package_name], py_modules=['unified_navigation_node','task_manager_node','detection','yolo_backend'], data_files=[('share/ament_index/resource_index/packages',['resource/nav']),('share/nav',['package.xml']),('share/nav/models',['models/products_best.pt']),('share/nav',['dataset.yaml'])], install_requires=['setuptools'], entry_points={'console_scripts':['unified_navigation=unified_navigation_node:main','task_manager=task_manager_node:main','kele_detection=detection:main']})
