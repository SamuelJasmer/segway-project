s = serial('COM4');
set(s,'BaudRate',9600);
fopen(s);
pause(10);

while(true)
   
    x = fscanf(s);
    disp(x);
    C = textscan(x,'%f'); 
    m = deal(C{:});
    
    mag_x = m(1,1);
    mag_y = m(2,1);
    mag_z = m(3,1);
    
    m_theta_x = m(4,1);
    m_theta_y = m(5,1);
    m_theta_z = m(6,1);
    
    m_theta_x = deg2rad (m_theta_x);
    m_theta_y = deg2rad (m_theta_y);
    m_theta_z = deg2rad (m_theta_z);
    
    magnitude_matrix = mag_x;
    theta_matrix = m_theta_x;
    
    hold on;
    %x = mag_x.*cos(m_theta_x);
    %y = mag_x.*sin(m_theta_x);
    
    x = mag_x;
    y = mag_y;
    
    clf;
    compass(x,y);
    drawnow;
    
end