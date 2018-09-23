#!/usr/bin/env julia

#=
	File name: filesystem_helpers.jl
    Author: Lukas Brunke
    Email: lukas.brunke@tuhh.de
    Julia Version: 0.4.7
=#


function get_most_recent_file()
	cd("/home/mpcubuntu/simulations/lukas")

	folders = []
	# Get all the folders
	for folder_string in readdir()
		if !contains(folder_string, ".")
			folders = [folders; folder_string]
		end
	end

	# Turn string into number
	folder_numbers = map(x->parse(Int64, x), folders)
	# Find most recent folder
	max_index = findmax(folder_numbers)[2]
	chosen_folder = folders[max_index]

	cd(chosen_folder)
	files = []
	time_index = []
	# Check all files in the folder
	for file in readdir()
		files = [files; file]
		time_index = [time_index; parse(Int64, split(file, "_", limit=2)[1])]
	end

	max_time_index = findmax(time_index)[2]
	chosen_file = files[max_time_index]

	return pwd() * "/" * chosen_file
end

function find_file(time_stamp_string)
	cd("/home/mpcubuntu/simulations/lukas")

	if length(time_stamp_string) == 4
		# Take most recently recorded file from the folder indicated by the 
		# first four digits
		cd(time_stamp_string)
		files = []
		time_index = []
		# Check all files in the folder
		for file in readdir()
			files = [files; file]
			time_index = [time_index; parse(Int64, split(file, "_", limit=2)[1])]
		end

		max_time_index = findmax(time_index)[2]
		chosen_file = files[max_time_index]

		return pwd() * "/" * chosen_file
	elseif length(time_stamp_string) == 10
		folder = time_stamp_string[1 : 4]
		cd(folder)

		file_string = time_stamp_string[5 : end]

		for file in readdir()
			if contains(file, file_string)
				return pwd() * "/" * file
			end
		end

		error("The specified string does not match any file.")
		 
	else 
		error("String not matching the folder or file structure.")
	end
end


function create_window_name(filename)
	splitted_string = split(filename, "/")
	last_bits = splitted_string[end - 1 : end]

	file_name = split(split(last_bits[2], ".")[1], "_")
	time_stamp = file_name[1][1 : 2] * ":" * file_name[1][3 : 4] * ":" * 
				 file_name[1][5 : 6]

	agent_str = ucfirst(file_name[2])
	index_str = file_name[3]
	mode_str = uppercase(file_name[4])
	init_str = ucfirst(file_name[5])

	month_day = last_bits[1][1 : 2] * "/" * last_bits[1][3 : 4]

	figure_string = agent_str * " " * index_str * " " * mode_str * " " * 
					init_str * " " * month_day * " " * time_stamp

	return figure_string 
end


function get_index(filename)
	if contains(filename, "agent_1")
		return 1
		# return parse(Int64, split(filename, "_")[end - 2])
	elseif contains(filename, "agent_2")
		return 2
	end
end


function get_most_recent(node_name, mode, initialization)
	cd("/home/mpcubuntu/simulations/lukas")

	folders = []
	# Get all the folders
	for folder_string in readdir()
		if !contains(folder_string, ".")
			folders = [folders; folder_string]
		end
	end

	# Turn string into number
	folder_numbers = map(x->parse(Int64, x), folders)
	# reverse the order, because we want the most recent one
	sorted_indeces = sortperm(folder_numbers)[end : - 1 : 1]

	for index in sorted_indeces
		cd(folders[index])
		files = []
		time_index = []
		# Check all files in the folder
		for file in readdir()
			if contains(file, mode) && contains(file, initialization) &&
				contains(file, node_name)
				files = [files; file]
				time_index = [time_index; parse(Int64, split(file, "_", limit=2)[1])]
			end
		end

		if size(files, 1) > 0
			max_time_index = findmax(time_index)[2]
			chosen_file = files[max_time_index]

			return pwd() * "/" * chosen_file
		end
			
		cd("..")
	end
end