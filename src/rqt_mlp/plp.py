class Plp():
    def __init__(self, path):
        import subprocess
        subprocess.Popen(path, shell=True)
        print path