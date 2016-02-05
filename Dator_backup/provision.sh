#!/usr/bin/env bash
apt-get update
apt-get upgrade -y

debconf-set-selections <<< 'mysql-server mysql-server/root_password password BrightBox1234'
debconf-set-selections <<< 'mysql-server mysql-server/root_password_again password BrightBox1234'

apt-get install -f --yes --force-yes libssl-dev libfreetype6-dev libffi-dev libmemcached-dev # required by numpy, scipy
apt-get install -f --yes --force-yes python-dev python-setuptools
apt-get install -f --yes --force-yes mysql-server
apt-get install -f --yes --force-yes mysql-client libmysqlclient-dev
apt-get install -f --yes --force-yes redis-server
#apt-get install -f --yes --force-yes mongodb
#apt-get install -f --yes --force-yes rabbitmq-server
#apt-get install -f --yes --force-yes python-pip
apt-get install -f --yes --force-yes r-base r-base-dev
apt-get install -f --yes --force-yes apache2 libapache2-mod-wsgi

apt-get install -f --yes --force-yes build-essential chrpath git-core libssl-dev libfontconfig1-dev libxft-dev
apt-get install -f --yes --force-yes phantomjs
apt-get install -f --yes --force-yes curl
apt-get install -f --yes --force-yes nodejs
apt-get install -f --yes --force-yes ntp

ln -s /usr/bin/nodejs /usr/bin/node
apt-get install -f --yes --force-yes npm

apt-get -f install --yes --force-yes

cd /vagrant/
echo "moved to project directory"

npm config set registry http://registry.npmjs.org/
npm cache clean -f
npm install -g n
n stable
npm install -g bower
npm install -g karma
npm install -g jasmine jasmine-core
npm install -g karma-jasmine karma-chrome-launcher karma-phantomjs-launcher
npm install -g karma-cli
npm install -g karma-coverage
npm install -g karma-junit-reporter

pip install --upgrade -d /usr/bin/ pip
pip install numpy==1.9.2
pip install -r /vagrant/bbdatabox/pip_dependencies.txt
python /vagrant/autodeploy/main.py render_local
easy_install requests==2.2.1

#mysql -uroot -pBrightBox1234 -e "create database db_dev"

#echo "If the database was already created, an exception may have been printed."


# OLD LIST OF LIBRARIES HERE

# fresh image needs:

# apt-get install =========

# git
# python-setuptools
# libmysqlclient-dev
# r-base r-base-dev #we may want to get these from cran
# python-dev
# libffi-dev
# libssl-dev
# libmemcached-dev
# libfreetype6-dev
# phantomjs

# optional:
# mongodb
# mysql-server
# redis-server
# rabbitmq-server
# apache2 libapache2-mod-wsgi


# easy_install =======
# pip

# pip install ======
# pip install -r bbdatabox/pip_dependencies.txt

# !! matplotlib, scipy     <==== issues with these guys



# worker config:  (remove not needed stuff, make upstart scripts)
#     sudo apt-get -y remove apache2 mongodb mysql-server rabbitmq-server mysql-server-5.5 mysql-server-core-5.5 mongodb-server redis-server; sudo apt-get -y autoremove
#     make proper bbox workers conf