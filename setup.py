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
        'Development Status :: 3 - Alpha',  # 4 - Beta, 5 - Production/Stable
        'Intended Audience :: Education',
        'Intended Audience :: End Users/Desktop',
        'Intended Audience :: Science/Research',
        'Programming Language :: Python :: 3',
        'Programming Language :: Python :: 3.7',
        'Topic :: Games/Entertainment :: Simulation'
    ],
    keywords='traffic simulation agent-based',
    packages=find_packages(exclude=['contrib', 'docs', 'tests']),
    python_requires='>=3.7',

    # TODO:
    # install_requires=['peppercorn'],
    # extras_require={'dev': ['check-manifest'], 'test': ['coverage']},
    # entry_points={'console_scripts': ['sample=sample:main']},
    # project_urls={'Bug Reports': 'https:...'}
)
