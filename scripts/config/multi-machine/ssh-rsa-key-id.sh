ssh-keygen -t rsa
eval $(ssh-agent)
ssh-add
ssh-copy-id odroid@10.0.0.1
