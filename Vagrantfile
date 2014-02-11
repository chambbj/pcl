# -*- mode: ruby -*-
# vi: set ft=ruby :

# Vagrantfile API/syntax version. Don't touch unless you know what you're doing!
VAGRANTFILE_API_VERSION = "2"

Vagrant.configure(VAGRANTFILE_API_VERSION) do |config|
  config.vm.box = "precise64"
  config.vm.hostname = "pcl-vagrant"
  config.vm.box_url = "http://files.vagrantup.com/precise64.box"
  config.vm.network :forwarded_port, guest: 80, host: 8080 
  config.vm.provider :virtualbox do |vb|
     vb.customize ["modifyvm", :id, "--memory", "4096"]
     vb.customize ["modifyvm", :id, "--cpus", "4"]   
     vb.customize ["modifyvm", :id, "--ioapic", "on"]
     vb.name = "pcl-vagrant"
#     vb.gui = true
   end  

  ppaRepos = [
#    "ppa:v-launchpad-jochen-sprickerhof-de/pcl"
#	  "ppa:apokluda/boost1.53",
#          "ppa:kalakris/eigen"
  ]

  packageList = [
    "git",
    "build-essential",
    "make",
    "pkg-config",
    "cmake",
#    "libpcl-all",
    "vim",
    "cmake-curses-gui",
    "libboost1.48-all-dev",
    "libeigen3-dev",
    "libvtk5-dev",
    "libflann-dev"
  ];

  if Dir.glob("#{File.dirname(__FILE__)}/.vagrant/machines/default/*/id").empty?
    pkg_cmd = ""
#    pkg_cmd << "apt-get update -qq; apt-get install -q -y python-software-properties; "
    pkg_cmd << "apt-get update -qq;"

    if ppaRepos.length > 0
      ppaRepos.each { |repo| pkg_cmd << "add-apt-repository -y " << repo << " ; " }
      pkg_cmd << "apt-get update -qq; "
    end

    # install packages we need
    pkg_cmd << "apt-get install -q -y " + packageList.join(" ") << " ; "
    config.vm.provision :shell, :inline => pkg_cmd

    scripts = [
      "pcl.sh"
    ];
    scripts.each { |script| config.vm.provision :shell, :path => "scripts/vagrant/" << script }
  end
end
