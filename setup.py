"""A setuptools based setup module."""

from os import path

from setuptools import setup, find_packages

HERE = path.abspath(path.dirname(__file__))
with open(path.join(HERE, 'README.md'), encoding='utf-8') as file:
    LONG_DESCRIPTION = file.read()

setup(
    name='tsim',
    version='0.0.1',
    description='Agent based traffic simulator.',
    long_description=LONG_DESCRIPTION,
    long_description_content_type='text/markdown',
    url='https://gitlab.com/eduardomezencio/tsim',
    author='Eduardo Mezêncio',
    author_email='eduardomezencio@protonmail.com',
    classifiers=[  # https://pypi.org/classifiers/
        'Development Status :: 3 - Alpha',
        'Intended Audience :: Education',
        'Intended Audience :: End Users/Desktop',
        'Intended Audience :: Science/Research',
        'Programming Language :: Python :: 3',
        'Programming Language :: Python :: 3.9',
        'Topic :: Games/Entertainment :: Simulation'
    ],
    keywords='traffic simulation agent-based',
    packages=find_packages(exclude=['contrib', 'docs', 'tests']),
    python_requires='>=3.9, <4',
    install_requires=[
        'aggdraw>=1.3.12',
        'bezier>=2021.2.12',
        'dataslots>=1.0.2',
        'fibonacci-heap-mod>=1.1',
        'intervaltree>=3.1.0',
        'numpy>=1.20.1',
        'orderedset>=2.0.3',
        'panda3d>=1.10.8',
        'Pillow>=8.1.2',
        'Rtree>=0.9.7'
    ],
    extras_require={
        'dev': [
            'flake8',
            'ipython',
            'isort',
            'jedi',
            'matplotlib',
            'pycodestyle',
            'pydocstyle',
            'pylint',
            'radon',
            'rope'
        ]  # , 'test': []
    },
    entry_points={
        'console_scripts': [
            'tsim-osm-cleaner = tsim.scripts.osm_cleaner:main',
            'tsim-osm-reader = tsim.scripts.osm_reader:main'
        ],
        'gui_scripts': [
            'tsim = tsim:main'
        ]
    }
)
