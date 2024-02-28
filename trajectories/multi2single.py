import os
import h5py

def merge_hdf5_files(source_dir, output_file):
    with h5py.File(output_file, 'w') as merged_hdf5:
        # Create the 'data' group in the merged file
        data_group = merged_hdf5.create_group('data')
        
        # Iterate through all files in the source directory
        for filename in sorted(os.listdir(source_dir)):
            if filename.endswith('.hdf5'):
                # Extract trajectory number from the file name
                trajectory_num = filename.split('_')[-1].split('.')[0]
                # Create a new group for this trajectory under 'data'
                trajectory_group = data_group.create_group(f'trajectory_{trajectory_num}')

                # Open the source HDF5 file
                with h5py.File(os.path.join(source_dir, filename), 'r') as source_hdf5:
                    # Copy all datasets from the source file to the new trajectory group
                    for dset_name in source_hdf5:
                        source_hdf5.copy(dset_name, trajectory_group)

                print(f'Merged {filename} into {trajectory_group.name}')

if __name__ == '__main__':
    # Set the directory containing your .hdf5 files
    source_directory = '/'  # or your specific path
    # Set the name of the output merged file
    output_filename = 'merged_trajectories.hdf5'
    # Call the function
    merge_hdf5_files(source_directory, output_filename)

