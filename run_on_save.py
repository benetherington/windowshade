import sys

# print(sys.argv)
# exit()

( #pylint: disable=unbalanced-tuple-unpacking
  executed_script,
  workspace_folder,
  file_dir_name,
  file_base_name
) = sys.argv

#      *** cmd start: py run_on_save.py c:\py-libraries\glass-cockpit c:\py-libraries\glass-cockpit run_on_save.py
print("                  ", end='')
print("executed_script".center(len(executed_script)), end='')
print("workspace_folder".center(len(workspace_folder)), end='')
print("file_dir_name".center(len(file_dir_name)), end='')
print("file_base_name".center(len(file_base_name)))

if file_base_name == executed_script:
  print("It me!")
  exit()

from shutil import copyfile
local_file_path = file_dir_name + "\\" + file_base_name
_, _, sub_directory = file_dir_name.partition(workspace_folder)
sub_directory = sub_directory[1:] # strip leading slash
remote_file_path = "E:\\" + sub_directory + "\\" + file_base_name

with open(local_file_path) as f:
  first_line = f.readline()

if first_line.strip() == "#autocopy":
  print(f"copy '{local_file_path}' to '{remote_file_path}'")
  copyfile(local_file_path, remote_file_path)
else:
  print(f"No autocopy! first_line: {first_line}")

