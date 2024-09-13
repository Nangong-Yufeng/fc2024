function update()
    local index = mission:get_current_nav_index()
    if index ~= nil then
        if param:get("TARGET_GET") == 1 then
            gcs:send_text(6,string.format("index:%f",index))
            if param:get("TARGET_NUM") == 1 then
                if index >= 1 then
                    param:set_and_save("TARGET_WAYPOINT",1)
                end
            elseif param:get("TARGET_NUM") == 2 then
                if index >= 1 then
                    param:set_and_save("TARGET_WAYPOINT",1)
                end
            elseif param:get("TARGET_NUM") == 3 then
                if index >= 1 then
                    param:set_and_save("TARGET_WAYPOINT",1)
                end
            end
        end
    end
    return update,700
end
return update,1000
