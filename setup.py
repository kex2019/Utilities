import setuptools

setuptools.setup(
    name="RoboticWareHouseUtils",
    version="0.0.1",
    author="Jonas & Johan",
    author_email="jonas@valfridsson.net",
    description="Utils for warehouse simulator",
    url="https://github.com/kex2019/Utilities",
    packages=["robotic_warehouse_utils"],
    install_requires=["numpy==1.14.2", "pandas==0.22.0"],
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: MIT License",
        "Operating System :: OS Independent",
    ],
)
