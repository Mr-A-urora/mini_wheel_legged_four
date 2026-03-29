function ak = get_lw_fitting()
    i=0.06;
    j=1;
    lw=zeros(1,30);
    k=0.06:0.005:0.205;
    while i<=0.2
        lw(j) = double(get_lw(i));
        i=i+0.005;
        j=j+1;
    end
    ak=polyfit(k,lw,2);

end
