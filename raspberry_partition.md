# Create the partition path

`sudo mkdir -p /var/cache/swap/`

# Set the size of the partition
## bs=64M is the block size, count=64 is the number of blocks, so the swap space size is bs*count=4096MB=4GB

`sudo dd if=/dev/zero of=/var/cache/swap/swap0 bs=64M count=64`

# Set permissions for this directory

`sudo chmod 0600 /var/cache/swap/swap0`

# Create the SWAP file

`sudo mkswap /var/cache/swap/swap0`

# Activate the SWAP file

`sudo swapon /var/cache/swap/swap0`

# Check if SWAP information is correct

`sudo swapon -s`

# TO REMOVE SWAP

`sudo swapoff /var/cache/swap/swap0`

`sudo rm /var/cache/swap/swap0`
