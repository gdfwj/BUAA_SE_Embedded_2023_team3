from django.urls import path
from patch_embedding import views
urlpatterns = [
    path('main/resetAll/', views.ResetAll.as_view(), name=''),
    path('main/update/', views.Update.as_view(), name=''),
    path('map/showAll/', views.ShowAll.as_view(), name=''),
    path('map/create/', views.Create_map.as_view(), name=''),
    path('map/save/', views.Save_map.as_view(), name=''),
    path('map/delete/', views.DeleteMap.as_view(), name=''),
    path('mark/show/', views.ShowMark.as_view(), name=''),
    path('mark/create/', views.CreateMark.as_view(), name=''),
    path('mark/save/', views.SaveMark.as_view(), name=''),
    path('service/init/', views.ServiceInit.as_view(), name=''),
    path('navigation/begin/', views.Navigation.as_view(), name=''),
    path('object/fetch/', views.Fetch.as_view(), name=''),
    path('control/voice/', views.VoiceChange.as_view(), name=''),

]
