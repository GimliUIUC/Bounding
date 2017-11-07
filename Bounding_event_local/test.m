


function TDpoint = test(t,book)

ind = find(t > book.touchdownTime,1,'last');
ind = ind(1);

TDpoint = book.touchdownList(ind);


end