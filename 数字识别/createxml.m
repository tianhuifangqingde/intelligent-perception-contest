function createxml(name,datatest) % name��������ļ�����data��matlab�еľ���(һ�㶼��float��ʽ�洢��)  
xdoc=com.mathworks.xml.XMLUtils.createDocument('opencv_storage');  
xroot=xdoc.getDocumentElement;  
%  
[m,n]=size(datatest);  
type=xdoc.createElement(name);  
xroot.appendChild(type);  
type.setAttribute('type_id','opencv-matrix')  
%  
rows=xdoc.createElement('rows');  
rows.appendChild(xdoc.createTextNode(sprintf('%d',m)));  
type.appendChild(rows);  
cols=xdoc.createElement('cols');  
cols.appendChild(xdoc.createTextNode(sprintf('%d',n)));  
type.appendChild(cols);  
dt=xdoc.createElement('dt');  
dt.appendChild(xdoc.createTextNode(sprintf('%s','f')));  
type.appendChild(dt);  
data=xdoc.createElement('data');  
data.appendChild(xdoc.createTextNode(sprintf('%f ',datatest)));  
type.appendChild(data);  
str=strcat(name,'.xml');  
xmlwrite(str,xdoc);  
end  