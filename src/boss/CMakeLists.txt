ADD_LIBRARY(boss_library SHARED
  stream.cpp stream.h
  bidirectional_serializer.cpp bidirectional_serializer.h
  blob.cpp blob.h     
  deserializer.cpp deserializer.h
  identifiable.cpp identifiable.h
  id_context.cpp id_context.h   
  id_placeholder.cpp id_placeholder.h
  json_message_parser.cpp json_message_parser.h
  json_message_writer.cpp json_message_writer.h
  json_object_parser.cpp json_object_parser.h
  json_object_writer.cpp json_object_writer.h
  logger.cpp logger.h
  message.cpp message.h
  message_data.cpp message_data.h
  message_parser.cpp message_parser.h
  message_writer.cpp message_writer.h
  object_data.cpp object_data.h
  object_parser.cpp object_parser.h
  object_writer.cpp object_writer.h
  serializable.cpp serializable.h
  serializer.cpp serializer.h
  serialization_context.cpp serialization_context.h
)

TARGET_LINK_LIBRARIES(boss_library 
  ${Boost_FILESYSTEM_LIBRARY} 
  ${Boost_SYSTEM_LIBRARY})
