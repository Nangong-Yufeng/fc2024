local function read_mission(file_name) --从文件中读取航点
   -- Open file
   file = assert(io.open(file_name), 'Could not open :' .. file_name)
   -- check header
   assert(string.find(file:read('l'),'QGC WPL 110') == 1, file_name .. ': incorrect format')
   -- clear any existing mission
   assert(mission:clear(), 'Could not clear current mission')
   -- read each line and write to mission
   local item = mavlink_mission_item_int_t()
   local index = 0
   local fail = false
   while true and not fail do
      local data = {}
      local line = file:read()
      if not line then
         break
      end
      local ret, _, seq, curr, frame, cmd, p1, p2, p3, p4, x, y, z, autocont = string.find(line, "^(%d+)%s+(%d+)%s+(%d+)%s+(%d+)%s+([-.%d]+)%s+([-.%d]+)%s+([-.%d]+)%s+([-.%d]+)%s+([-.%d]+)%s+([-.%d]+)%s+([-.%d]+)%s+(%d+)")
      if not ret then
         fail = true
         break
      end
      if tonumber(seq) ~= index then
         fail = true
         break
      end
      item:seq(tonumber(seq))
      item:frame(tonumber(frame))
      item:command(tonumber(cmd))
      item:param1(tonumber(p1))
      item:param2(tonumber(p2))
      item:param3(tonumber(p3))
      item:param4(tonumber(p4))
      if mission:cmd_has_location(tonumber(cmd)) then
         item:x(math.floor(tonumber(x)*10^7))
         item:y(math.floor(tonumber(y)*10^7))
      else
         item:x(math.floor(tonumber(x)))
         item:y(math.floor(tonumber(y)))
      end
      item:z(tonumber(z))
      if not mission:set_item(index,item) then
         mission:clear() -- clear part loaded mission
         fail = true
         break
      end
      index = index + 1
   end
   if fail then
      mission:clear()  --clear anything already loaded
      error(string.format('failed to load mission at seq num %u', index))
   end
   gcs:send_text(0, string.format("Loaded %u mission items", index))
end
function update()
   if param:get("TARGET_GET") == 1 then
      if vehicle:get_mode() == 11 then
         if param:get("TARGET_NUM") == 1 then
            read_mission('./scripts/way1.txt')
            param:set_and_save("TARGET_AUTO",1)
         elseif param:get("TARGET_NUM") == 2 then
            read_mission('./scripts/way2.txt')
            param:set_and_save("TARGET_AUTO",1)
         elseif param:get("TARGET_NUM") == 3 then
            read_mission('./scripts/way3.txt')
            param:set_and_save("TARGET_AUTO",1)
            
         end
      end
   else
      return update,1000
   end
end
return update,5000
