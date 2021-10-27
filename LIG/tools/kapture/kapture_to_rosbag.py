import sys 
REPO_ROOT_PATH = '/home/euigon/kapture'
sys.path.insert(0, REPO_ROOT_PATH)
import kapture
import kapture.io.csv as csv

dataset_path = '/media/euigon/Samsung_T5/bags/NAVER_LABS/GangnamStation_B1_release_mapping/GangnamStation/B1/release/mapping/'

tar_handlers = csv.get_all_tar_handlers(dataset_path)
kapture_data = csv.kapture_from_dir(dataset_path, tar_handlers=tar_handlers)

print(kapture_data.records_depth)    

tar_handlers.close()