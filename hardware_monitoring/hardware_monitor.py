#!/usr/bin/env python3

##################################################
# based on https://www.thepythoncode.com/article/get-hardware-system-information-python
##################################################

import psutil
import GPUtil
import platform
import time


def get_size(bytes, suffix="B"):
    """
    Scale bytes to its proper format
    e.g:
        1253656 => '1.20MB'
        1253656678 => '1.17GB'
    """
    factor = 1024
    for unit in ["", "K", "M", "G", "T", "P"]:
        if bytes < factor:
            return f"{bytes:.2f}{unit}{suffix}"
        bytes /= factor


class HardwareMonitor():
    def __init__(self):
        pass

    def get_platform_info(self):
        """Returns basic plattform information"""

        uname = platform.uname()
        platform_info = {
            'System': uname.system,
            'PC-Name': uname.node,
            'Release': uname.release,
            'Version': uname.version,
            'Machine': uname.machine,
            'Processor': uname.processor
        }

        return platform_info

    def get_cpu_info(self):
        """Returns amount of CPU cores and actual detailed CPU load"""

        cpufreq = psutil.cpu_freq()
        cpu_info = {
            'Phys. cores': psutil.cpu_count(logical=False),
            'Total cores': psutil.cpu_count(logical=True),
            'Max Frequency': f'{cpufreq.max:.2f}Mhz',
            'Min Frequency': f'{cpufreq.min:.2f}Mhz',
            'Current Frequency': f'{cpufreq.current:.2f}Mhz'
        }

        for i, percentage in enumerate(psutil.cpu_percent(percpu=True, interval=1)):
            cpu_info.update({f'Core {i}': f'{percentage}%'})

        cpu_info.update({'Total CPU Usage': f'{psutil.cpu_percent()}%'})

        return cpu_info

    def get_fast_cpu_info(self):
        cpufreq = psutil.cpu_freq()
        cpu_info = {
            'Phys. cores': psutil.cpu_count(logical=False),
            'Total cores': psutil.cpu_count(logical=True),
            'Max Frequency': f'{cpufreq.max:.2f}Mhz',
            'Min Frequency': f'{cpufreq.min:.2f}Mhz',
            'Current Frequency': f'{cpufreq.current:.2f}Mhz'
        }

        cpu_info.update({'Total CPU Usage': f'{psutil.cpu_percent()}%'})

        return cpu_info

    def get_memory_info(self):
        svmem = psutil.virtual_memory()
        swap = psutil.swap_memory()
        memory_info = {
            'Total memory': get_size(svmem.total),
            'Available memory': get_size(svmem.available),
            'Used memory': get_size(svmem.used),
            'Percentage memory in use': f'{svmem.percent}%',
            'Total swap': get_size(swap.total),
            'Free swap': get_size(swap.free),
            'Used swap': get_size(swap.used),
            'Percentage swap in use': f'{swap.percent}%'
        }

        return memory_info

    def get_gpu_info(self):
        gpus = GPUtil.getGPUs()
        gpu_info = {}

        for i in range(len(gpus)):
            gpu_dict = {
                f'GPU{i+1}': {
                    'ID': gpus[i].id,
                    'Name': gpus[i].name,
                    'Load': f"{gpus[i].load*100}%",
                    'Free memory': f"{gpus[i].memoryFree}MB",
                    'Used memory': f"{gpus[i].memoryUsed}MB",
                    'Total memory': f"{gpus[i].memoryTotal}MB",
                    'Temperature': f"{gpus[i].temperature} °C",
                    'UUID': gpus[i].uuid
                }}

            gpu_info.update(gpu_dict)

        return gpu_info

    def get_base_info(self):
        base_info = {
            'System': self.get_platform_info(),
            'CPU': self.get_fast_cpu_info(),
            'Memory': self.get_memory_info(),
            'GPUs': self.get_gpu_info()
        }

        return base_info

    def get_performance_info(self):
        cpufreq = psutil.cpu_freq()
        svmem = psutil.virtual_memory()
        swap = psutil.swap_memory()
        gpus = GPUtil.getGPUs()

        gpu_info = {}
        for i in range(len(gpus)):
            gpu_dict = {
                f'GPU{i+1}': {
                    'ID': gpus[i].id,
                    'Load': f"{gpus[i].load*100}%",
                    'Used memory': f"{gpus[i].memoryUsed}MB",
                    # 'Temperature': f"{gpus[i].temperature} °C",
                }}

            gpu_info.update(gpu_dict)

        perf_info = {
            'CPU': {
                'Total CPU usage': f'{psutil.cpu_percent()}%',
                'Current Frequency': f'{cpufreq.current:.2f}Mhz'
            },
            'Memory': {
                'Used memory': get_size(svmem.used),
                '% memory in use': f'{svmem.percent}%',
                'Used swap': get_size(swap.used),
                r'% swap in use': f'{swap.percent}%'
            },
            'GPU': gpu_info
        }

        return perf_info


def test():
    HM = HardwareMonitor()

    print(HM.get_platform_info())
    print(HM.get_cpu_info())
    print(HM.get_memory_info())
    print(HM.get_gpu_info())


def speed_test():
    HM = HardwareMonitor()

    start = time.perf_counter()
    print(HM.get_platform_info())
    print(
        f'Calculation time for system info: {round(time.perf_counter() - start, 6)}s\n\n')

    start = time.perf_counter()
    print(HM.get_cpu_info())
    print(
        f'Calculation time for CPU info: {round(time.perf_counter() - start, 6)}s\n\n')

    start = time.perf_counter()
    print(HM.get_fast_cpu_info())
    print(
        f'Calculation time for CPU info without core loads: {round(time.perf_counter() - start, 6)}s\n\n')

    start = time.perf_counter()
    print(HM.get_memory_info())
    print(
        f'Calculation time for memory info: {round(time.perf_counter() - start, 6)}s\n\n')

    start = time.perf_counter()
    print(HM.get_gpu_info())
    print(
        f'Calculation time for gpu info: {round(time.perf_counter() - start, 6)}s\n\n')


if __name__ == '__main__':
    HM = HardwareMonitor()
    print(HM.get_base_info())
