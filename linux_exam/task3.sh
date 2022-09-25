#!/usr/bin/env bash

cd /home/user/catkin_ws/src/linux_exam/this/is/my/linux/exam
rm *
touch exam1.py exam2.py exam3.py
chmod u+rwx exam1.py
chmod g+rx exam1.py
chmod g-w exam1.py
chmod o+r exam1.py
chmod o-wx exam1.py

chmod u+rx exam2.py
chmod u-w exam2.py
chmod g-rwx exam2.py
chmod o+x exam2.py
chmod o-wr exam2.py

chmod u+w exam3.py
chmod u-rx exam3.py
chmod g+r exam3.py
chmod g-wx exam3.py
chmod o+x exam3.py
chmod o-rw exam3.py
