# ~/.profile: executed by the command interpreter for login shells.
# This file is not read by bash(1), if ~/.bash_profile or ~/.bash_login
# exists.
# see /usr/share/doc/bash/examples/startup-files for examples.
# the files are located in the bash-doc package.

# the default umask is set in /etc/profile; for setting the umask
# for ssh logins, install and configure the libpam-umask package.
#umask 022

# if running bash
if [ -n "$BASH_VERSION" ]; then
    # include .bashrc if it exists
    if [ -f "$HOME/.bashrc" ]; then
	. "$HOME/.bashrc"
    fi
fi

# set PATH so it includes user's private bin if it exists
if [ -d "$HOME/bin" ] ; then
    PATH="$HOME/bin:$PATH"
fi

# start up server when user logins in 
source ~/barc/scripts/login.sh

# Make sure crontab is updated
#crontab -u odroid /home/odroid/barc/scripts/crondump

# Make sure to have a team name already registered
source /home/odroid/team_name.sh

export DATOR_SERVER='http://localhost:8000'
export PYTHONPATH=$PYTHONPATH:/usr/local/lib/python2.7/site-packages
/usr/bin/autossh -M 16000 -R 14000:localhost:22 mpc@104.131.132.100 -N -i /home/odroid/barc/.ssh/id_rsa -f
