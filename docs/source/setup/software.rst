Software Installation
=====================

Install Vehicle Software
------------------------

.. code-block:: bash

  git clone --recursive https://github.com/<osu-uwrt>/riptide.git
  cd riptide
  ./scripts/prep-workspace
  cd ~/osu-uwrt/riptide_ws/src
  ./scripts/install-ceres
  ./scripts/install-ros
  source ~/.bashrc
  ./scripts/setup-workspace
  source ~/.bashrc

.. note::
  Replace <osu-uwrt> with your GitHub username when cloning.

Install Documentation Software
------------------------------

.. code-block:: bash

  ./scripts/install-sphinx
  cd ~/osu-uwrt/riptide_ws/src/docs
  make html
