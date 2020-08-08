#!/usr/bin/env python3


import yaml

class configData:
    def __init__(self):
        with open("hyperparameters.yaml", 'r') as stream:
            try:
                config = yaml.safe_load(stream)
            except yaml.YAMLError as exc:
                print(exc)

        self.buffer_size = int(float(config.get('bufferParameters', {}).get('maxSize')))
        self.batch_size = int(config.get('bufferParameters', {}).get('batchSize'))
        
        self.env = config.get('policy', {}).get('env')
        self.hiddenSizes = list(config.get('policy', {}).get('hiddenSizes'))
        self.steps_per_epoch = int(config.get('policy', {}).get('steps_per_epoch'))
        self.epochs = int(config.get('policy', {}).get('epochs'))
        self.gamma = float(config.get('policy', {}).get('gamma'))
        self.polyak = float(config.get('policy', {}).get('polyak'))
        self.lr = float(config.get('policy', {}).get('lr'))
        self.alpha = float(config.get('policy', {}).get('alpha'))
        self.start_steps = int(config.get('policy', {}).get('start_steps'))
        self.max_ep_len = int(config.get('policy', {}).get('max_ep_len'))
        self.seed = int(config.get('policy', {}).get('seed'))



'''
data = configData()
print("steps_per_epoch su: ", data.steps_per_epoch)
print("epochs su: ", data.epochs)
print("gamma su: ", data.gamma)
print("polyak su: ", data.polyak)
print("lr su: ", data.lr)
print("alpha su: ", data.alpha)
print("start_steps su: ", data.start_steps)
print("max_ep_len su: ", data.max_ep_len)
print("save_freq su: ", data.save_freq)
print("hiddenSizes ", data.hiddenSizes)
'''
