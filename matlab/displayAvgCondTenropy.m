function [ output ] = displayAvgCondTenropy(par_strFilename, par_figureOffset, par_figure_title)

    avgConditEntropy = dlmread(par_strFilename);
    figure(par_figureOffset+1);
    plot(avgConditEntropy);
    title(['Average conditional entropy during learning', ' | ', par_figure_title], 'Interpreter', 'none');
    xlabel('learning steps');
    ylabel('average conditional entropy')

end