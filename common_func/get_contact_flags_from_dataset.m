function contact_flags = get_contact_flags_from_dataset(file_path)
    newStr = split(file_path,'/');newStr2 = split(newStr{end},'.');
    dataset_name = strcat('../',newStr2{1}, '_contact.mat');
    dataset = load(dataset_name,'contact_flags_receding');
    contact_flags = dataset.contact_flags_receding;
end