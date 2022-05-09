function [stop,isterminal,direction] = eventsFcn(t,x,T,xf)

    e=(x(1:3)-xf(1:3));
    if norm(e)<.1
        stop =0;
        disp(T-t)
    else 
        stop =1;
    end


  
  
  isterminal = 1;  % Halt integration 
  direction = 0;   % The zero can be approached from either direction
end