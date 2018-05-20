function [ output_args ] = function_HighQualityPlot( figure, fontType, fontSize )
%This function helps in creating high-quality plots for publication purposes
%   

axes = findall(allchild(figure), 'Type', 'Axes');
textarrows = findall(allchild(figure), 'Type', 'TextArrow');

set(axes, 'XGrid', 'on', 'YGrid','on');
set(axes,'FontName',fontType, 'FontSize', fontSize);
set(textarrows,'FontName',fontType, 'FontSize', fontSize);

end

