function h = add_channel_to_array( h, cdata, c )

h = [ h; double(reshape(cdata(:,:,c),[],1)) ];

end

