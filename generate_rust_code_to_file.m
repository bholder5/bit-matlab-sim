function generate_rust_code_to_file(filename, varargin)
    rust_code = generate_rust_code(varargin{:});

    % Open a file for writing
    fid = fopen(filename, 'wt');

    % Check if the file was opened successfully
    if fid == -1
        error('Failed to open the file for writing.');
    end

    % Write the Rust code to the file
    fprintf(fid, '%s', rust_code);

    % Close the file
    fclose(fid);
end
