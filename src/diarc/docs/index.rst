.. diarc documentation master file, created by
   sphinx-quickstart on Wed Aug 20 19:38:45 2014.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

DiArc Documentation!
====================

.. toctree::
   :hidden:

   terminology  
   topology
..   :maxdepth: 2


Building
--------



Building the Documentation
--------------------------
First you need ``sphinx`` installed, on Ubuntu:

.. code-block:: bash

    $ sudo apt-get install python-sphinx

On other platforms use pip:

.. code-block:: bash

    $ sudo pip install Sphinx

You have to have built the package first, then you mush source the resulting devel or install space:

.. code-block:: bash

    $ source /path/to/space/setup.bash

Then from the capabilities source folder you can build the docs:

.. code-block:: bash

    $ cd docs
    $ make html

The resulting docs will be generated to ``doc/.build/html/index.html``.


Running the Tests
-----------------


Running the Code
----------------


Indices and tables
==================

* :ref:`genindex`
* :ref:`modindex`
* :ref:`search`

