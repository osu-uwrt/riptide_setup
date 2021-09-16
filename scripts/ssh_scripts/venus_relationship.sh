mkdir ~/.ssh/

if [ "$(grep ssh-agent ~/.bashrc)" = "" -a "$USER" = "ros" ]
then
	echo "eval \"\$(ssh-agent -s)\" > /dev/null" >> ~/.bashrc
	echo "ssh-add -D > /dev/null" >> ~/.bashrc
	echo "ssh-add ~/.ssh/sshkey_venus > /dev/null" >> ~/.bashrc
	eval $(ssh-agent -s)
fi

cd
cd .ssh

ssh-keygen -b 2048 -t rsa -f /tmp/sshkey_venus -q -N ""
ssh-copy-id -i /tmp/sshkey_venus ros@venus

mv /tmp/sshkey_venus ~/.ssh/sshkey_venus
mv /tmp/sshkey_venus.pub ~/.ssh/sshkey_venus.pub

ssh-add -D
ssh-add sshkey_venus

ssh-keyscan venus >> ~/.ssh/known_hosts 
