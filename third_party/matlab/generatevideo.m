clear;
close all;

X=1:100;
Y=1:100;
N=length(X);

for i = 1:N
    figure(1);
    plot(X(i),Y(i),'o');
    xlim([0 100]);
    ylim([0 100]);
    hold off;
    F(i) = getframe(gcf);
    
end
% create the video writer with 1 fps
writerObj = VideoWriter('myVideo.avi');
writerObj.FrameRate = 10;
% set the seconds per image
% open the video writer
open(writerObj);
% write the frames to the video
for i=1:length(F)
    % convert the image to a frame
    frame = F(i) ;
    writeVideo(writerObj, frame);
end
% close the writer object
close(writerObj);