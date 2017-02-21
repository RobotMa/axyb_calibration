function plotProbResults(error_1, error_2, point, opt)
%%
if strcmp(opt, 'boxplot')
    
    error_1_perm = permute(error_1, [3, 1, 2]);
    error_2_perm = permute(error_2, [3, 1, 2]);
    
    for i = 1: size(error_1_perm, 3)
        figure
        boxplot(error_1_perm(:,:,i), point);
    end

    for i = 1: size(error_2_perm, 3)
        figure

    end 
    
elseif strcmp(opt, 'lineplot')
    
    Err1_Avg = sum(error_1, 3)/size(error_1, 3);
    Err2_Avg = sum(error_2, 3)/size(error_2, 3);
    
    figure
    subplot(2,1,1)
    plot(point, Err1_Avg(:,1),'o')
    hold on
    plot(point, Err2_Avg(:,1),'*')
    hold on
    plot(point, Err1_Avg(:,1),'b')
    hold on
    plot(point, Err2_Avg(:,1),'r')
    yl1 = ylabel('$\bf E_{R_{X}}$');
    set(yl1,'FontSize',14,'Interpreter','latex');
    len1 = legend('$Prob1$','$Prob2$');
    set(len1,'FontSize',14,'Interpreter','latex');
    
    subplot(2,1,2)
    plot(point, Err1_Avg(:,2),'o')
    hold on
    plot(point, Err2_Avg(:,2),'*')
    hold on
    plot(point, Err1_Avg(:,2),'b')
    hold on
    plot(point, Err2_Avg(:,2),'r')
    yl2 = ylabel('$\bf E_{R_{Y}}$');
    set(yl2,'FontSize',14,'Interpreter','latex');
    
    figure
    subplot(2,1,1)
    plot(point, Err1_Avg(:,3),'o')
    hold on
    plot(point, Err2_Avg(:,3),'*')
    hold on
    plot(point, Err1_Avg(:,3),'b')
    hold on
    plot(point, Err2_Avg(:,3),'r')
    yl4 = ylabel('$\bf E_{t_{X}}$');
    set(yl4,'FontSize',14,'Interpreter','latex');
    len4 = legend('$Prob1$','$Prob2$');
    set(len4,'FontSize',12,'Interpreter','latex');
    
    subplot(2,1,2)
    plot(point, Err1_Avg(:,4),'o')
    hold on
    plot(point, Err2_Avg(:,4),'*')
    hold on
    plot(point, Err1_Avg(:,4),'b')
    hold on
    plot(point, Err2_Avg(:,4),'r')
    yl6 = ylabel('$\bf E_{t_{Y}}$');
    set(yl6,'FontSize',14,'Interpreter','latex');
    %len6 = legend('$E1_{t_{Z}}$','$E2_{t_{X}}$');
    %set(len6,'FontSize',12,'Interpreter','latex');
    
end

end