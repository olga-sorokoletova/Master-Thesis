import os, sys
#os.system("pip install pandas")
import pandas as pd
import argparse

sys.path.append(os.getenv("MARRTINO_APPS_HOME")+"/navigation")

def get_label(doctor_name):

    df = pd.read_csv('providers.csv')
    cmd_path = sys.path[-1]+ "/er_cmd.nav"

    record = df.loc[df['Doctor Name'] == doctor_name]

    status = record['Status'].item()
    if status == "home":
        write_path(cmd_path, "telecom")
    elif status == "hospital":
        office = record['Office'].item()
        write_path(cmd_path, "door" + str(office) + "out")
    else:
        print("Doctor is not available today.")

def write_path(filename, target_label):
    cmd = "gotoLabel(\"%s\")" % target_label
    with open(filename, 'wt') as f:
        f.write(cmd)

if __name__=='__main__':

    parser = argparse.ArgumentParser(description='navigator')
    parser.add_argument('doctor_name', type=str, help='Name of a physician to communicate with')
    args = parser.parse_args()
    get_label(args.doctor_name)

