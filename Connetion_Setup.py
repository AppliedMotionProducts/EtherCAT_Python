# Find all the availabe adapters and find EtherCAT slave

# Find your adapter id
import pysoem

adapters = pysoem.find_adapters()

for i, adapter in enumerate(adapters):
   print('Adapter {}'.format(i))
   print('  {}'.format(adapter.name))
   print('  {}'.format(adapter.desc))


# Find EtherCAT slave device
master = pysoem.Master()

# once you find the correct adapter address open the master to get communication with the slave
master.open("\\Device\\NPF_{'your adapter id}")

if master.config_init() > 0:
   for device in master.slaves:
      print(f'Found Device {device.name}')
else:
   print('no device found')


master.state
if master.config_init() > 0:
    print("Found", master.config_init(), "slaves")

