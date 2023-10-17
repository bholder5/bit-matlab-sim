function rust_code = generate_rust_code(varargin)
    nl = newline;
    rust_code = "extern crate nalgebra as na;" + nl + nl;
    
    for k = 1:nargin
        matrix = varargin{k};
        [rows, cols] = size(matrix);
        
        % Create the type name based on the matrix index (e.g., AMat1, AMat2, ...)
        type_name = "AMat" + num2str(k);
        
        
        % Define the type
        rust_code = rust_code + "pub type " + type_name + " = na::SMatrix<f64, " + num2str(rows) + ", " + num2str(cols) + ">;"+nl;
        
        % Define the init function for the matrix
        init_func_name = "init_" + type_name + "_pos";
        rust_code = rust_code + nl +"pub fn " + init_func_name + "() -> " + type_name + " {" + nl;
        rust_code = rust_code + "    let a = " + type_name + "::from_row_slice(&[" + nl;
        
        % Format matrix values
        for i = 1:rows
            for j = 1:cols
                rust_code = rust_code + sprintf("%.16f,", matrix(i, j));
                if j == cols
                    rust_code = rust_code + nl;
                end
            end
        end
        rust_code = rust_code + "    ]);" + nl;
        rust_code = rust_code + "    return a;" + nl;
        rust_code = rust_code + "}" + nl + nl;
    end
end
