from math import atan2, asin, sqrt

M_PI=3.1415926535

class Logger:
    def __init__(self, filename, headers=["e", "e_dot", "e_int", "stamp"]):
        self.filename = filename

        with open(self.filename, 'w') as file:
            header_str=""

            for header in headers:
                header_str+=header
                header_str+=", "
            
            header_str+="\n"
            
            file.write(header_str)


    def log_values(self, values_list):

        with open(self.filename, 'a') as file:
            vals_str=""

            for value in values_list:
                vals_str+=str(value)
                vals_str+=", "
            
            vals_str+="\n"
            
            file.write(vals_str)
            

    def save_log(self):
        pass

class FileReader:
    def __init__(self, filename):
        
        self.filename = filename
        
        
    def read_file(self):
        
        read_headers=False

        table=[]
        headers=[]
        with open(self.filename, 'r') as file:
            # Skip the header line

            if not read_headers:
                for line in file:
                    values=line.strip().split(',')

                    for val in values:
                        if val=='':
                            break
                        headers.append(val.strip())

                    read_headers=True
                    break
            
            next(file)
            
            # Read each line and extract values
            for line in file:
                values = line.strip().split(',')
                
                row=[]                
                array=[]
                is_arr_element = False
                
                for val in values:
                    if val=='':
                        break
                    if val.startswith("["):
                        is_arr_element = True
                        val = val[1:] # Remove open bracket
                    if val.endswith("]"):
                        is_arr_element = False
                        val = val[:-1]
                        array.append(float(val.strip()))
                        row.append(array)
                        continue
                    if is_arr_element:
                        array.append(float(val.strip()))
                    else:
                        row.append(float(val.strip()))

                table.append(row)
        
        return headers, table


# Convert quaternion to euler and return yaw
def euler_from_quaternion(quat):
    """
    Convert quaternion (w in last place) to euler roll, pitch, yaw.
    quat = [x, y, z, w]
    """
    x = quat.x
    y = quat.y
    z = quat.z
    w = quat.w
    
    yaw = atan2(2*(w*z+x*y), 1-2*(y*y+z*z))
    return yaw


