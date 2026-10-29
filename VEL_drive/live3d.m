function live3d(u)
    if u(3) >= 0, return; end
    persistent hf hl
    if isempty(hf) || ~ishandle(hf)
        hf = figure('Name','Live 3-D trajectory');
        hl = plot3(nan, nan, nan, 'b-', 'LineWidth', 1.5);
        hold on; grid on; box on;
        xlabel('N (m)'); ylabel('E (m)'); zlabel('Up (m)');
        view(45, 22);
    end
    xd = get(hl,'XData'); yd = get(hl,'YData'); zd = get(hl,'ZData');
    set(hl, 'XData',[xd u(1)], 'YData',[yd u(2)], 'ZData',[zd -u(3)]);
    drawnow limitrate;
end