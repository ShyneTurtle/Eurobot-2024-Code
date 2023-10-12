env = DefaultEnvironment()

def before_upload(source, target, env):
    print("before_upload")
    
    # env.Execute("platformio device monitor -b 1200 -p /dev/ttyACM0")


env.AddPreAction("upload", before_upload)